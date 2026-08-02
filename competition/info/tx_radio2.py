#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
信息波渐进抗干扰接收入口。

本文件是现场运行入口；GNU Radio 主图在 tx_radio2_flow.py，
可视化源文件在 tx_radio2.grc。这样重新 Generate GRC 时，不会覆盖本文件开头
最重要的抗干扰调参区。
"""

from __future__ import annotations

import signal
import sys
from distutils.version import StrictVersion

from PyQt5 import Qt

from tx_radio2_flow import tx_radio2_flow
from tx_radio2_tunes import ANTIJAM_PROFILE


# =============================================================================
# ★★★★★ 抗干扰参数调试区（现场优先只改这里）★★★★★
# =============================================================================
#
# 用哪个档？改 tx_radio2_tunes.py 里的 ANTIJAM_PROFILE，解码器读的是同一份，
# 不需要任何环境变量。下面 ANTIJAM_PROFILES 是各档的具体数值。
#
PLUTO_URI = "ip:192.168.3.1"  # SDR-B：信息波接收机
TARGET_FREQ_HZ = 433_200_000   # 默认红方；f3 会按 /team 通过 RPC 覆盖
SAMPLE_RATE_HZ = 1_000_000
SPS = 47
SENSITIVITY = 1.5628
GAIN_MU = 0.175
FREQ_ERROR = 0.0048
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_PORT = 8081

# RF bandwidth / gain 在 Pluto 前端生效；改档位后必须重启本程序。
# FIR cutoff / transition 在 ADC 后生效，不能挽救已经削顶的 IQ。
ANTIJAM_PROFILES = {
    "baseline": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
        "fir_window": "hamming",
        "dc_blocker_length": 0,
    },
    "balanced": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "dc_blocker_length": 32,
    },
    "strong": {
        "rf_bandwidth_hz": 540_000,
        "rx_gain_db": 25.0,
        "fir_cutoff_hz": 255_000.0,
        "fir_transition_hz": 15_000.0,
        "fir_window": "blackman_harris",
        "dc_blocker_length": 32,
    },
}
# =============================================================================


def _selected_profile() -> dict:
    if ANTIJAM_PROFILE not in ANTIJAM_PROFILES:
        choices = ", ".join(sorted(ANTIJAM_PROFILES))
        raise SystemExit(
            f"tx_radio2_tunes.ANTIJAM_PROFILE={ANTIJAM_PROFILE!r} 不认识；"
            f"可选：{choices}"
        )
    return dict(ANTIJAM_PROFILES[ANTIJAM_PROFILE])


def _build_flowgraph(profile: dict):
    """
    优先使用本仓库带构造参数的 tx_radio2_flow。

    若现场从 GRC 3.8 重新 Generate 成无参 top block，则退化为先构造再调用
    自动生成的 setter；tx_radio2.py 的调参区仍不会被覆盖。DC 是否旁路和 FIR
    window 属于拓扑属性，重新 Generate 后应以 GRC 图为准并留意启动警告。
    """
    kwargs = {
        "pluto_uri": PLUTO_URI,
        "target_freq": TARGET_FREQ_HZ,
        "samp_rate": SAMPLE_RATE_HZ,
        "rf_bandwidth": profile["rf_bandwidth_hz"],
        "rx_gain": profile["rx_gain_db"],
        "target_sens": SENSITIVITY,
        "fir_cutoff": profile["fir_cutoff_hz"],
        "fir_transition": profile["fir_transition_hz"],
        "fir_window": profile["fir_window"],
        "dc_blocker_length": profile["dc_blocker_length"],
        "udp_ip": UDP_IP,
        "udp_port": UDP_PORT,
        "rpc_port": RPC_PORT,
        "gain_mu": GAIN_MU,
        "freq_error": FREQ_ERROR,
    }
    try:
        return tx_radio2_flow(**kwargs)
    except TypeError as exc:
        print(f"[警告] 检测到 GRC 重新生成的无参 flowgraph：{exc}")
        print("[警告] 将通过 setter 应用数值；DC/FIR window 请同时核对 tx_radio2.grc。")
        tb = tx_radio2_flow()
        setter_values = {
            "set_samp_rate": SAMPLE_RATE_HZ,
            "set_target_freq": TARGET_FREQ_HZ,
            "set_target_sens": SENSITIVITY,
            "set_rf_bandwidth": profile["rf_bandwidth_hz"],
            "set_rx_gain": profile["rx_gain_db"],
            "set_fir_cutoff": profile["fir_cutoff_hz"],
            "set_fir_transition": profile["fir_transition_hz"],
        }
        for setter_name, value in setter_values.items():
            setter = getattr(tb, setter_name, None)
            if callable(setter):
                setter(value)
            else:
                print(f"[警告] flowgraph 缺少 {setter_name}，对应参数未动态应用")
        return tb


def main() -> None:
    profile = _selected_profile()
    print("=" * 72)
    print(f"信息波 tx_radio2 抗干扰档位：{ANTIJAM_PROFILE}")
    print(
        f"Pluto={PLUTO_URI} | RF BW={profile['rf_bandwidth_hz']/1e3:.0f} kHz | "
        f"gain={profile['rx_gain_db']:.1f} dB"
    )
    print(
        f"FIR={profile['fir_cutoff_hz']/1e3:.0f}/"
        f"{profile['fir_transition_hz']/1e3:.0f} kHz "
        f"{profile['fir_window']} | DC={profile['dc_blocker_length']}"
    )
    print(f"SPS={SPS} | Sens={SENSITIVITY} | UDP={UDP_IP}:{UDP_PORT} | RPC={RPC_PORT}")
    print("若原始频谱削顶：先降 gain；数字 FIR 无法恢复已削顶的 IQ。")
    print("=" * 72)

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        from gnuradio import gr

        style = gr.prefs().get_string("qtgui", "style", "raster")
        Qt.QApplication.setGraphicsSystem(style)

    qapp = Qt.QApplication(sys.argv)
    tb = _build_flowgraph(profile)
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
