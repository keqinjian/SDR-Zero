#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio5_flow.py —— f5 接收主图（小白说明）
==========================================

相对 f4：优先 symbol_sync(M&M+MMSE) 做时钟恢复；并提供 apply_runtime_tune()
给解码器热切换 RF 带宽/增益/FIR。无 symbol_sync 时自动回退 clock_recovery_mm。
"""

from __future__ import annotations

import math

from gnuradio import analog, digital, filter

from tx_radio2_flow import tx_radio2_flow


class tx_radio5_flow(tx_radio2_flow):
    """
    保留 Pluto、复数 FIR、频谱和 XMLRPC；解调链改为：

      FIR → Quadrature Demod → [可选平滑] → symbol_sync(M&M+MMSE) → Slicer

    若当前 GNU Radio 无 symbol_sync_ff，则回退到 clock_recovery_mm_ff。
    SPS / Sensitivity / 空口格式不改变。运行时可通过 apply_runtime_tune 调 RF/FIR。
    """

    def __init__(
        self,
        *,
        sps: int = 47,
        post_demod_dc_length: int = 0,
        smooth_samples: int = 1,
        clock_omega_relative_limit: float = 0.005,
        prefer_symbol_sync: bool = True,
        **kwargs,
    ):
        self.sps = int(sps)
        self.post_demod_dc_length = int(post_demod_dc_length)
        self.smooth_samples = max(1, int(smooth_samples))
        self.clock_gain_mu = float(kwargs["gain_mu"])
        self.clock_freq_error = float(kwargs["freq_error"])
        self.clock_omega_relative_limit = float(clock_omega_relative_limit)
        self.prefer_symbol_sync = bool(prefer_symbol_sync)
        self.clock_backend = "pending"
        self.runtime_tune_name = "init"

        super().__init__(**kwargs)
        self.setWindowTitle("info RX adaptive f5")

        self.disconnect(
            (self.low_pass_filter_0, 0), (self.digital_gfsk_demod_0, 0)
        )
        self.disconnect(
            (self.digital_gfsk_demod_0, 0), (self.blocks_udp_sink_0, 0)
        )

        self.quadrature_demod_0 = analog.quadrature_demod_cf(
            1.0 / self.target_sens
        )

        self.post_demod_dc_0 = None
        if self.post_demod_dc_length > 0:
            self.post_demod_dc_0 = filter.dc_blocker_ff(
                self.post_demod_dc_length, True
            )

        self.post_demod_smooth_0 = None
        if self.smooth_samples > 1:
            taps = [1.0 / self.smooth_samples] * self.smooth_samples
            self.post_demod_smooth_0 = filter.fir_filter_fff(1, taps)

        self.clock_recovery_0 = self._make_clock_recovery()
        self.binary_slicer_0 = digital.binary_slicer_fb()

        chain = [self.low_pass_filter_0, self.quadrature_demod_0]
        if self.post_demod_dc_0 is not None:
            chain.append(self.post_demod_dc_0)
        if self.post_demod_smooth_0 is not None:
            chain.append(self.post_demod_smooth_0)
        chain.extend(
            [self.clock_recovery_0, self.binary_slicer_0, self.blocks_udp_sink_0]
        )
        for source, sink in zip(chain, chain[1:]):
            self.connect((source, 0), (sink, 0))

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

    def apply_runtime_tune(
        self,
        name,
        rx_gain,
        rf_bandwidth,
        fir_cutoff,
        fir_transition,
    ) -> str:
        """
        供解码器自适应控制器调用。只改 RF/FIR，不改 SPS/Sens/解调拓扑。
        XMLRPC 只接受简单类型，故参数全部展开传递。
        """
        self.set_rx_gain(float(rx_gain))
        self.set_rf_bandwidth(int(rf_bandwidth))
        self.set_fir_cutoff(float(fir_cutoff))
        self.set_fir_transition(float(fir_transition))
        self.runtime_tune_name = str(name)
        return self.runtime_tune_name
