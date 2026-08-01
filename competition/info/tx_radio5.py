#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""信息波 f5 通用自适应接收入口。"""

from __future__ import annotations

import os
import signal
import sys
from distutils.version import StrictVersion

from PyQt5 import Qt

from tx_radio5_flow import tx_radio5_flow
from tx_radio5_tunes import AUTO_TUNE_ORDER, RUNTIME_TUNES


PROFILE = os.environ.get("INFO_F5_PROFILE", "auto").strip().lower()

PLUTO_URI = "ip:192.168.3.1"
TARGET_FREQ_HZ = 433_200_000
SAMPLE_RATE_HZ = 1_000_000
SPS = 47
SENSITIVITY = 1.5628
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_PORT = 8081

# 启动时解调拓扑参数（symbol_sync 等）；运行时 RF/FIR 由 RUNTIME_TUNES 切换。
# auto 仅选择启动拓扑，真正现场适配由 info_decoder_f5 的控制器完成。
BOOT_PROFILES = {
    "auto": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 60_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
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
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
        "prefer_symbol_sync": True,
        "runtime_tune": "open",
    },
    "weak_fixed": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 40.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 3,
        "gain_mu": 0.10,
        "freq_error": 0.0,
        "omega_relative_limit": 0.01,
        "prefer_symbol_sync": True,
        "runtime_tune": "weak_boost",
    },
}


def selected_boot_profile() -> dict:
    if PROFILE not in BOOT_PROFILES:
        choices = ", ".join(sorted(BOOT_PROFILES))
        raise SystemExit(f"未知 INFO_F5_PROFILE={PROFILE!r}；可选：{choices}")
    return dict(BOOT_PROFILES[PROFILE])


def build_flowgraph(profile: dict) -> tx_radio5_flow:
    tb = tx_radio5_flow(
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
        prefer_symbol_sync=profile["prefer_symbol_sync"],
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
    print(f"信息波 f5 profile={PROFILE}")
    print(
        f"Pluto={PLUTO_URI} | RF BW={profile['rf_bandwidth_hz']/1e3:.0f}kHz | "
        f"gain={profile['rx_gain_db']:.1f}dB | "
        f"FIR={profile['fir_cutoff_hz']/1e3:.0f}/"
        f"{profile['fir_transition_hz']/1e3:.0f}kHz"
    )
    print(
        f"SPS={SPS} Sens={SENSITIVITY} | 平滑={profile['smooth_samples']} | "
        f"symbol_sync优先={profile['prefer_symbol_sync']} | "
        f"UDP={UDP_IP}:{UDP_PORT}"
    )
    print(
        "运行时自适应档: "
        + ", ".join(AUTO_TUNE_ORDER)
        + "（由 info_decoder_f5 经 XMLRPC 切换）"
    )
    print("=" * 72)

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        from gnuradio import gr

        style = gr.prefs().get_string("qtgui", "style", "raster")
        Qt.QApplication.setGraphicsSystem(style)

    qapp = Qt.QApplication(sys.argv)
    tb = build_flowgraph(profile)
    print(f"时钟后端={tb.get_clock_backend()} | 初始tune={tb.get_runtime_tune_name()}")
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
