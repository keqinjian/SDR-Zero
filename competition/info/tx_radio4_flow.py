#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f4 弱信号信息波接收主图：在 f3 前端后展开 GFSK 解调链。"""

from __future__ import annotations

from gnuradio import analog, digital, filter

from tx_radio2_flow import tx_radio2_flow


class tx_radio4_flow(tx_radio2_flow):
    """
    保留 tx_radio2 的 Pluto、复数 FIR、频谱和 XMLRPC，只替换 GFSK 解调部分。

    显式链路允许在 FM 判决域消除残余直流（主要来自载频偏差），并在
    M&M 时钟恢复前做很短的平滑。SPS、Sensitivity 和空口格式均不改变。
    """

    def __init__(
        self,
        *,
        sps: int = 47,
        post_demod_dc_length: int = 0,
        smooth_samples: int = 1,
        clock_omega_relative_limit: float = 0.005,
        **kwargs,
    ):
        self.sps = int(sps)
        self.post_demod_dc_length = int(post_demod_dc_length)
        self.smooth_samples = max(1, int(smooth_samples))
        self.clock_gain_mu = float(kwargs["gain_mu"])
        self.clock_freq_error = float(kwargs["freq_error"])
        self.clock_omega_relative_limit = float(clock_omega_relative_limit)

        super().__init__(**kwargs)
        self.setWindowTitle("info RX weak-signal f4")

        # 移除 f3 的黑盒 gfsk_demod；保留父类对象以兼容父类结构，但不再连接。
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

        gain_omega = 0.25 * self.clock_gain_mu * self.clock_gain_mu
        omega = self.sps * (1.0 + self.clock_freq_error)
        self.clock_recovery_0 = digital.clock_recovery_mm_ff(
            omega,
            gain_omega,
            0.5,
            self.clock_gain_mu,
            self.clock_omega_relative_limit,
        )
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

    def _apply_demod_sensitivity(self, sensitivity):
        sens = float(sensitivity)
        if sens != 0.0:
            self.quadrature_demod_0.set_gain(1.0 / sens)

