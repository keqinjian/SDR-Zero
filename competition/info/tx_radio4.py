#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio4.py —— 信息波 f4 的 GNU Radio 接收入口（小白说明）
========================================================

本文件只负责“收音机”：Pluto 收 433MHz → 鉴频/时钟 → 把 0/1 经 UDP 14346 送给
info_decoder_f4.py。协议解析不在这里做。

环境变量 INFO_F4_PROFILE：
  weak_antijam — 弱信号抗干扰（默认）
  baseline     — 接近 f3 基线，用来对比有没有退步

官方空口旋钮固定：SPS=47，Sensitivity=1.5628，采样率 1MHz。
"""

from __future__ import annotations

import os
import signal
import sys
from distutils.version import StrictVersion

from PyQt5 import Qt

from tx_radio4_flow import tx_radio4_flow


PROFILE = os.environ.get("INFO_F4_PROFILE", "weak_antijam").strip().lower()

PLUTO_URI = "ip:192.168.3.1"
TARGET_FREQ_HZ = 433_200_000
SAMPLE_RATE_HZ = 1_000_000
SPS = 47
SENSITIVITY = 1.5628
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_PORT = 8081

# baseline 与 f3 基线一致；weak_antijam 只调整接收机，不改变任何空口参数。
# 判决域 DC blocker 会把超过其延迟长度的连续码元当作直流消除；
# 固定 Header 含 12 个连续 0，因此正式 profile 中保持关闭。
PROFILES = {
    "baseline": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
        "fir_window": "hamming",
        "complex_dc_length": 0,
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
    },
    "weak_antijam": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 40.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 5,
        "gain_mu": 0.10,
        "freq_error": 0.0,
        "omega_relative_limit": 0.01,
    },
}


def selected_profile() -> dict:
    if PROFILE not in PROFILES:
        choices = ", ".join(sorted(PROFILES))
        raise SystemExit(f"未知 INFO_F4_PROFILE={PROFILE!r}；可选：{choices}")
    return dict(PROFILES[PROFILE])


def build_flowgraph(profile: dict) -> tx_radio4_flow:
    return tx_radio4_flow(
        pluto_uri=PLUTO_URI,
        target_freq=TARGET_FREQ_HZ,
        samp_rate=SAMPLE_RATE_HZ,
        rf_bandwidth=profile["rf_bandwidth_hz"],
        rx_gain=profile["rx_gain_db"],
        target_sens=SENSITIVITY,
        fir_cutoff=profile["fir_cutoff_hz"],
        fir_transition=profile["fir_transition_hz"],
        fir_window=profile["fir_window"],
        dc_blocker_length=profile["complex_dc_length"],
        udp_ip=UDP_IP,
        udp_port=UDP_PORT,
        rpc_port=RPC_PORT,
        gain_mu=profile["gain_mu"],
        freq_error=profile["freq_error"],
        sps=SPS,
        post_demod_dc_length=profile["post_demod_dc_length"],
        smooth_samples=profile["smooth_samples"],
        clock_omega_relative_limit=profile["omega_relative_limit"],
    )


def main() -> None:
    profile = selected_profile()
    print("=" * 72)
    print(f"信息波 f4 profile={PROFILE}")
    print(
        f"Pluto={PLUTO_URI} | RF BW={profile['rf_bandwidth_hz']/1e3:.0f}kHz | "
        f"gain={profile['rx_gain_db']:.1f}dB | "
        f"FIR={profile['fir_cutoff_hz']/1e3:.0f}/"
        f"{profile['fir_transition_hz']/1e3:.0f}kHz"
    )
    print(
        f"SPS={SPS} Sens={SENSITIVITY} | 判决域DC={profile['post_demod_dc_length']} "
        f"平滑={profile['smooth_samples']} | UDP={UDP_IP}:{UDP_PORT}"
    )
    print("若 raw IQ 顶平，数字处理无法恢复：应降低 gain 或增加模拟滤波。")
    print("=" * 72)

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        from gnuradio import gr

        style = gr.prefs().get_string("qtgui", "style", "raster")
        Qt.QApplication.setGraphicsSystem(style)

    qapp = Qt.QApplication(sys.argv)
    tb = build_flowgraph(profile)
    tb.start()
    tb.show()

    def sig_handler(sig=None, frame=None):
        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    def quitting():
        tb.stop()
        tb.wait()

    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()


if __name__ == "__main__":
    main()
