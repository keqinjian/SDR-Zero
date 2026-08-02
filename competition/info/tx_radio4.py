#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio4.py —— 信息波 f4 的 GNU Radio 接收入口（小白说明）
========================================================

本文件只负责“收音机”：Pluto 收 433MHz → 鉴频/时钟 → 把 0/1 经 UDP 14346 送给
info_decoder_f4.py。协议解析不在这里做。

【怎么启动】平时不用单独跑它，`python3 info_decoder_f4.py` 会自动把它拉起来。
            要单独看频谱窗时才：`python3 tx_radio4.py`

【要调档位/增益】改 tx_radio4_tunes.py，不要改这里，也不需要任何环境变量。
【要调下面这些】Pluto 地址、频点、UDP/RPC 端口，就在本文件的调参面板里改。

官方空口旋钮固定：SPS=47，Sensitivity=1.5628，采样率 1MHz，任何时候都不要动。
"""

from __future__ import annotations

import signal
import sys
from distutils.version import StrictVersion

from PyQt5 import Qt

from tx_radio4_flow import tx_radio4_flow
import tx_radio4_tunes


# =============================================================================
# ★★★★★ 调参面板（射频档位在 tx_radio4_tunes.py）★★★★★
# =============================================================================

# 红方 433.2MHz；蓝方 433.92MHz。解码器会根据 /team 话题自动切频，
# 这里只是没连解码器、单独跑本文件时的初始频点。
PLUTO_URI = "ip:192.168.3.1"
TARGET_FREQ_HZ = 433_200_000
SAMPLE_RATE_HZ = 1_000_000
SPS = 47
SENSITIVITY = 1.5628
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_PORT = 8081
# =============================================================================

# 档位表和"用哪个档"都在 tx_radio4_tunes.py，解码器也读同一份，不会两边打架。
PROFILE = tx_radio4_tunes.PROFILE
selected_profile = tx_radio4_tunes.selected_profile


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
    print(
        "反之，若频谱图整体压在 -55dBFS 以下且看不到削顶，说明增益不足、"
        "信号淹在量化噪声里：改 tx_radio4_tunes.RX_GAIN_OVERRIDE 往上加 5~10dB 再看。"
    )
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
