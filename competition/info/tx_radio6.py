#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio6.py —— 信息波 f6 接收入口（小白说明）
============================================

在 f5 基础上增加：
  - 慢偏置扣除 + Gauss 匹配滤波
  - 硬比特 UDP 14346 + 软符号 float UDP 14347
  - IQ/FM 探头（给解码器做 AGC）
  - 可选 IQ 录波，事后用 info_f6_iq_replay.py 离线复盘

【怎么启动】平时不用单独跑它，`python3 info_decoder_f6.py` 会自动把它拉起来。
            要单独看频谱窗时才：`python3 tx_radio6.py`

【要调档位/AGC】改 tx_radio6_tunes.py，不需要任何环境变量。
与 info_decoder_f6 配对；SPS=47 / Sens=1.5628 不变，任何时候都不要动。
"""

from __future__ import annotations

import signal
import sys
from distutils.version import StrictVersion

from PyQt5 import Qt

from tx_radio6_flow import tx_radio6_flow
from tx_radio6_tunes import (
    AUTO_TUNE_ORDER,
    BOOT_PROFILE,
    RUNTIME_TUNES,
    SOFT_UDP_PORT,
)


# 开机拓扑选哪个：唯一来源是 tx_radio6_tunes.BOOT_PROFILE，解码器读的是同一份。
PROFILE = BOOT_PROFILE

# =============================================================================
# ★★★★★ 调参面板（射频档位与 AGC 在 tx_radio6_tunes.py）★★★★★
# =============================================================================
PLUTO_URI = "ip:192.168.3.1"
TARGET_FREQ_HZ = 433_200_000
SAMPLE_RATE_HZ = 1_000_000
SPS = 47
SENSITIVITY = 1.5628
BT = 0.35  # 高斯匹配滤波的 BT 积，官方 GFSK 调制用的就是 0.35
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_PORT = 8081

# 录 IQ 原始数据用于离线复盘：填一个路径字符串（如 "/tmp/info_f6.c64"）就开启，
# 填 None 关闭。注意 1MHz 复数采样约 8MB/s，别录太久把磁盘写满。
# 录完用：python3 competition/info/info_f6_iq_replay.py /tmp/info_f6.c64
IQ_RECORD_PATH = None

# 是否往 14347 发软符号。关掉后解码器只能走硬比特路径。
ENABLE_SOFT_UDP = True

# 慢偏置扣除的收敛系数。它替代了会破坏 Header 长 0 的短 DC blocker：
# 越小跟得越慢但越不会吃掉数据，1e-5 相当于约 10 万个采样的时间常数。
# 只有在日志里看到鉴频均值长期严重偏离 0 时才需要调大。
BIAS_ALPHA = 1e-5
# =============================================================================

BOOT_PROFILES = {
    # 信息波 Carson 带宽是 ±270kHz（h=23.4 的宽带 FSK），cutoff 必须比它大，
    # 否则一开机就在削自己的边带。过渡带保持 20kHz 让阻带在 295kHz 起，
    # 压住三级干扰。完整推导见 tx_radio6_tunes.py 里的注释。
    "auto": {
        "rf_bandwidth_hz": 700_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 275_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
        "prefer_symbol_sync": True,
        "runtime_tune": "balanced",
    },
    "fixed_balanced": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 60_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
        "prefer_symbol_sync": True,
        "runtime_tune": "balanced",
    },
    "baseline": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
        "fir_window": "hamming",
        "complex_dc_length": 0,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
        "prefer_symbol_sync": True,
        "runtime_tune": "open",
    },
}


def selected_boot_profile() -> dict:
    if PROFILE not in BOOT_PROFILES:
        choices = ", ".join(sorted(BOOT_PROFILES))
        raise SystemExit(
            f"tx_radio6_tunes.BOOT_PROFILE={PROFILE!r} 不认识；可选：{choices}"
        )
    return dict(BOOT_PROFILES[PROFILE])


def build_flowgraph(profile: dict) -> tx_radio6_flow:
    tb = tx_radio6_flow(
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
        bt=BT,
        bias_alpha=BIAS_ALPHA,
        clock_omega_relative_limit=profile["omega_relative_limit"],
        prefer_symbol_sync=profile["prefer_symbol_sync"],
        soft_udp_port=SOFT_UDP_PORT,
        iq_record_path=IQ_RECORD_PATH,
        enable_soft_udp=ENABLE_SOFT_UDP,
    )
    tune_name = profile.get("runtime_tune", "balanced")
    if tune_name in RUNTIME_TUNES:
        tune = RUNTIME_TUNES[tune_name]
        tb.apply_runtime_tune(
            tune_name,
            tune["rx_gain_db"],
            tune["rf_bandwidth_hz"],
            tune["fir_cutoff_hz"],
            tune["fir_transition_hz"],
        )
    return tb


def main() -> None:
    profile = selected_boot_profile()
    print("=" * 72)
    print(f"信息波 f6 profile={PROFILE}")
    print(
        f"Pluto={PLUTO_URI} | RF BW={profile['rf_bandwidth_hz']/1e3:.0f}kHz | "
        f"gain={profile['rx_gain_db']:.1f}dB | "
        f"FIR={profile['fir_cutoff_hz']/1e3:.0f}/"
        f"{profile['fir_transition_hz']/1e3:.0f}kHz"
    )
    print(
        f"SPS={SPS} Sens={SENSITIVITY} BT={BT} | bias_alpha={BIAS_ALPHA} | "
        f"hard UDP={UDP_IP}:{UDP_PORT} soft={SOFT_UDP_PORT}"
    )
    print("运行时档: " + ", ".join(AUTO_TUNE_ORDER))
    if IQ_RECORD_PATH:
        print(f"IQ 录波: {IQ_RECORD_PATH}")
    print("=" * 72)

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        from gnuradio import gr

        style = gr.prefs().get_string("qtgui", "style", "raster")
        Qt.QApplication.setGraphicsSystem(style)

    qapp = Qt.QApplication(sys.argv)
    tb = build_flowgraph(profile)
    print(
        f"时钟后端={tb.get_clock_backend()} | tune={tb.get_runtime_tune_name()} | "
        f"soft_udp={ENABLE_SOFT_UDP}"
    )
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
