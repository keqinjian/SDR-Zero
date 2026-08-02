#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio6_flow.py —— f6 接收主图（小白说明）
==========================================

链路：FIR → 鉴频 → 慢偏置扣除 → Gauss 匹配滤波 → symbol_sync
     → 硬比特 UDP + 软符号 UDP
     → IQ/FM/soft 探头（给 AGC）+ 可选 IQ 录波

慢偏置时间常数很长，避免把 Header 里连续多个 0 当成“直流”滤掉。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

from gnuradio import analog, blocks, digital, filter, gr
from gnuradio.filter import firdes

from tx_radio2_flow import tx_radio2_flow
from tx_radio6_tunes import AGC_GAIN_MAX_DB, AGC_GAIN_MIN_DB


def _float_stats(samples) -> dict:
    count = len(samples)
    if count == 0:
        return {"count": 0, "mean": 0.0, "rms": 0.0, "min": 0.0, "max": 0.0}
    total = 0.0
    power = 0.0
    min_v = float("inf")
    max_v = float("-inf")
    for value in samples:
        v = float(value)
        total += v
        power += v * v
        if v < min_v:
            min_v = v
        if v > max_v:
            max_v = v
    mean = total / count
    return {
        "count": int(count),
        "mean": float(mean),
        "rms": float(math.sqrt(power / count)),
        "min": float(min_v),
        "max": float(max_v),
    }


# ADC 削顶是按 I/Q 两路各自到轨判定的，不是看复数模长：
# 只要 |I| 或 |Q| 贴到满量程就已经失真了。单个 max 值容易被一次脉冲干扰
# 带偏，所以额外统计“到轨样本占比”，让上层能要求“持续削顶”才降增益。
CLIP_AXIS_LEVEL = 0.98


def _complex_stats(samples) -> dict:
    count = len(samples)
    if count == 0:
        return {
            "count": 0,
            "mean_abs": 0.0,
            "rms": 0.0,
            "max_abs": 0.0,
            "max_axis": 0.0,
            "clip_frac": 0.0,
            "mean_i": 0.0,
            "mean_q": 0.0,
        }
    abs_sum = 0.0
    power = 0.0
    max_abs = 0.0
    max_axis = 0.0
    clipped = 0
    real_sum = 0.0
    imag_sum = 0.0
    for sample in samples:
        real_sum += sample.real
        imag_sum += sample.imag
        axis = max(abs(sample.real), abs(sample.imag))
        if axis > max_axis:
            max_axis = axis
        if axis >= CLIP_AXIS_LEVEL:
            clipped += 1
        magnitude = abs(sample)
        abs_sum += magnitude
        power += magnitude * magnitude
        if magnitude > max_abs:
            max_abs = magnitude
    return {
        "count": int(count),
        "mean_abs": float(abs_sum / count),
        "rms": float(math.sqrt(power / count)),
        "max_abs": float(max_abs),
        "max_axis": float(max_axis),
        "clip_frac": float(clipped) / float(count),
        "mean_i": float(real_sum / count),
        "mean_q": float(imag_sum / count),
    }


class tx_radio6_flow(tx_radio2_flow):
    """
    FIR → QuadDemod → 慢偏置扣除 → Gauss 匹配滤波 → symbol_sync →
    soft float UDP + hard bit UDP；附 IQ/FM 探头与可选 IQ 录波。
    """

    def __init__(
        self,
        *,
        sps: int = 47,
        bt: float = 0.35,
        bias_alpha: float = 1e-5,
        match_span_symbols: int = 4,
        clock_omega_relative_limit: float = 0.005,
        prefer_symbol_sync: bool = True,
        soft_udp_port: int = 14347,
        probe_decim: int = 200,
        iq_record_path: Optional[str] = None,
        enable_soft_udp: bool = True,
        **kwargs,
    ):
        self.sps = int(sps)
        self.bt = float(bt)
        self.bias_alpha = float(bias_alpha)
        self.match_span_symbols = max(1, int(match_span_symbols))
        self.clock_gain_mu = float(kwargs["gain_mu"])
        self.clock_freq_error = float(kwargs["freq_error"])
        self.clock_omega_relative_limit = float(clock_omega_relative_limit)
        self.prefer_symbol_sync = bool(prefer_symbol_sync)
        self.soft_udp_port = int(soft_udp_port)
        self.probe_decim = max(1, int(probe_decim))
        self.iq_record_path = iq_record_path
        self.enable_soft_udp = bool(enable_soft_udp)
        self.clock_backend = "pending"
        self.runtime_tune_name = "init"
        self.udp_ip = str(kwargs.get("udp_ip", "127.0.0.1"))
        self.agc_gain_db = float(kwargs.get("rx_gain", 35.0))

        super().__init__(**kwargs)
        self.setWindowTitle("info RX near-optimal f6")
        self.agc_gain_db = float(self.rx_gain)
        # 父类构造参数里的 udp_ip 未挂到实例；用 kwargs 保留的副本。
        if "udp_ip" in kwargs:
            self.udp_ip = str(kwargs["udp_ip"])

        self.disconnect(
            (self.low_pass_filter_0, 0), (self.digital_gfsk_demod_0, 0)
        )
        self.disconnect(
            (self.digital_gfsk_demod_0, 0), (self.blocks_udp_sink_0, 0)
        )

        self.quadrature_demod_0 = analog.quadrature_demod_cf(
            1.0 / self.target_sens
        )

        # 慢速偏置：τ ≈ 1/(alpha*fs) ；alpha=1e-5 @1e6 → ~0.1s ≫ Header 连续0。
        self.bias_iir_0 = filter.single_pole_iir_filter_ff(self.bias_alpha)
        self.bias_sub_0 = blocks.sub_ff(1)

        ntaps = self.match_span_symbols * self.sps + 1
        match_taps = list(
            firdes.gaussian(1.0, float(self.sps), self.bt, ntaps)
        )
        # 归一化能量，避免改变判决幅度尺度过大
        energy = math.sqrt(sum(t * t for t in match_taps)) or 1.0
        match_taps = [t / energy for t in match_taps]
        self.match_filter_0 = filter.fir_filter_fff(1, match_taps)

        self.clock_recovery_0 = self._make_clock_recovery()
        self.binary_slicer_0 = digital.binary_slicer_fb()

        # 硬 bit 仍走原 UDP 14346
        self.connect(
            (self.low_pass_filter_0, 0), (self.quadrature_demod_0, 0)
        )
        self.connect((self.quadrature_demod_0, 0), (self.bias_sub_0, 0))
        self.connect((self.quadrature_demod_0, 0), (self.bias_iir_0, 0))
        self.connect((self.bias_iir_0, 0), (self.bias_sub_0, 1))
        self.connect((self.bias_sub_0, 0), (self.match_filter_0, 0))
        self.connect((self.match_filter_0, 0), (self.clock_recovery_0, 0))
        self.connect((self.clock_recovery_0, 0), (self.binary_slicer_0, 0))
        self.connect((self.binary_slicer_0, 0), (self.blocks_udp_sink_0, 0))

        self.blocks_soft_udp_sink_0 = None
        if self.enable_soft_udp:
            self.blocks_soft_udp_sink_0 = blocks.udp_sink(
                gr.sizeof_float,
                str(self.udp_ip),
                int(self.soft_udp_port),
                1472,
                True,
            )
            self.connect(
                (self.clock_recovery_0, 0), (self.blocks_soft_udp_sink_0, 0)
            )

        # 探头
        self.iq_probe_decim = blocks.keep_one_in_n(
            gr.sizeof_gr_complex, self.probe_decim
        )
        self.iq_probe = blocks.vector_sink_c()
        self.fm_probe_decim = blocks.keep_one_in_n(
            gr.sizeof_float, self.probe_decim
        )
        self.fm_probe = blocks.vector_sink_f()
        self.soft_probe_decim = blocks.keep_one_in_n(
            gr.sizeof_float, max(1, self.probe_decim // 10)
        )
        self.soft_probe = blocks.vector_sink_f()

        self.connect(
            (self.iio_pluto_source_0, 0),
            (self.iq_probe_decim, 0),
            (self.iq_probe, 0),
        )
        self.connect(
            (self.quadrature_demod_0, 0),
            (self.fm_probe_decim, 0),
            (self.fm_probe, 0),
        )
        self.connect(
            (self.clock_recovery_0, 0),
            (self.soft_probe_decim, 0),
            (self.soft_probe, 0),
        )

        self.iq_file_sink = None
        if self.iq_record_path:
            path = Path(self.iq_record_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.iq_file_sink = blocks.file_sink(
                gr.sizeof_gr_complex, str(path), False
            )
            self.connect((self.iio_pluto_source_0, 0), (self.iq_file_sink, 0))

    def _make_clock_recovery(self):
        omega = self.sps * (1.0 + self.clock_freq_error)
        gain_omega = 0.25 * self.clock_gain_mu * self.clock_gain_mu
        if self.prefer_symbol_sync and hasattr(digital, "symbol_sync_ff"):
            try:
                loop_bw = -math.log(
                    (self.clock_gain_mu + gain_omega) / (-2.0) + 1.0
                )
                max_dev = self.clock_omega_relative_limit * self.sps
                block = digital.symbol_sync_ff(
                    digital.TED_MUELLER_AND_MULLER,
                    omega,
                    loop_bw,
                    1.0,
                    1.0,
                    max_dev,
                    1,
                    digital.constellation_bpsk().base(),
                    digital.IR_MMSE_8TAP,
                    128,
                    [],
                )
                self.clock_backend = "symbol_sync_mm_mmse"
                return block
            except Exception:
                pass
        self.clock_backend = "clock_recovery_mm"
        return digital.clock_recovery_mm_ff(
            omega,
            gain_omega,
            0.5,
            self.clock_gain_mu,
            self.clock_omega_relative_limit,
        )

    def _apply_demod_sensitivity(self, sensitivity):
        sens = float(sensitivity)
        if sens != 0.0:
            self.quadrature_demod_0.set_gain(1.0 / sens)

    def get_clock_backend(self) -> str:
        return self.clock_backend

    def get_runtime_tune_name(self) -> str:
        return self.runtime_tune_name

    def get_agc_gain(self) -> float:
        return float(self.agc_gain_db)

    def apply_agc_gain(self, gain_db) -> float:
        gain = max(AGC_GAIN_MIN_DB, min(AGC_GAIN_MAX_DB, float(gain_db)))
        self.set_rx_gain(gain)
        self.agc_gain_db = gain
        return gain

    def apply_runtime_tune(
        self,
        name,
        rx_gain,
        rf_bandwidth,
        fir_cutoff,
        fir_transition,
    ) -> str:
        self.set_rx_gain(float(rx_gain))
        self.agc_gain_db = float(rx_gain)
        self.set_rf_bandwidth(int(rf_bandwidth))
        self.set_fir_cutoff(float(fir_cutoff))
        self.set_fir_transition(float(fir_transition))
        self.runtime_tune_name = str(name)
        return self.runtime_tune_name

    def get_iq_stats(self) -> dict:
        data = tuple(self.iq_probe.data())
        try:
            self.iq_probe.reset()
        except Exception:
            pass
        return _complex_stats(data)

    def get_fm_stats(self) -> dict:
        data = tuple(self.fm_probe.data())
        try:
            self.fm_probe.reset()
        except Exception:
            pass
        stats = _float_stats(data)
        if stats["count"]:
            # quad_demod 输出 ≈ freq_offset * 2π / (sens * samp_rate) 的近似反推
            stats["estimated_cfo_hz"] = float(
                stats["mean"]
                * self.target_sens
                * self.samp_rate
                / (2.0 * math.pi)
            )
        else:
            stats["estimated_cfo_hz"] = 0.0
        return stats

    def get_soft_stats(self) -> dict:
        data = tuple(self.soft_probe.data())
        try:
            self.soft_probe.reset()
        except Exception:
            pass
        return _float_stats(data)
