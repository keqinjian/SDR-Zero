#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio_szu.py —— info_decoder_f3_SZU.py 的配套收音机（自带流图·可无界面）
================================================================================

【这个文件是什么】
它就是 tx_radio2.py + tx_radio2_flow.py 两个文件合并成的独立版本，解调链
**逐块相同**，但去掉了对 tx_radio2_tunes.py 的依赖，参数全部由命令行传入。
这样交付给别的队只需要拷两个文件：

    info_decoder_f3_SZU.py   ← 解码 + 转发原始帧到下游
    tx_radio_szu.py          ← 本文件，Pluto 收 + 解调 + 把裸比特灌给上面那个

【正常用法：不用手动跑它】
info_decoder_f3_SZU.py 的看门狗会自动把它拉起来，也会在 UDP 断流时重启它。
所以现场只要：插上 SDR → `python3 info_decoder_f3_SZU.py` → 下游就有数据了。

【想单独跑（调试频谱时）】
    python3 tx_radio_szu.py --pluto-uri ip:192.168.2.1
然后把 info_decoder_f3_SZU.py 里的 ENABLE_GRC_WATCHDOG 改成 False，
两个终端各跑一个，互不干扰。

【解调链】
    Pluto ─→ [DC blocker 可选] ─→ 低通 FIR ─→ gfsk_demod(SPS=47) ─→ UDP 裸比特
              └─ 原始 IQ 频谱          └─ 滤波后 IQ 频谱（有界面时才画）

【没有图形界面也能跑】
检测不到 PyQt5 或没有 DISPLAY 时自动转成无界面模式，只是看不到频谱窗，
解调和 UDP 输出完全不受影响。赛场用 ssh 登录跑也没问题。
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading

from gnuradio import blocks, digital, filter, gr
from gnuradio.filter import firdes
import iio

try:
    from xmlrpc.server import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer
except ImportError:  # 极老的 Python2 风格环境
    from SimpleXMLRPCServer import (  # type: ignore
        SimpleXMLRPCRequestHandler,
        SimpleXMLRPCServer,
    )


# =============================================================================
# ★★★★★ 调参面板 ★★★★★
#
# 注意：这些只是"没有传命令行参数时"的默认值。正常由 info_decoder_f3_SZU.py
# 通过命令行传进来，所以**优先改那个文件**，不要两边各改一份改出不一致。
# =============================================================================

# Pluto 的地址。USB 直连时默认网段通常是 192.168.2.1；
# 如果 `iio_info -u ip:192.168.2.1` 能出信息就说明这个地址是对的。
DEFAULT_PLUTO_URI = "ip:192.168.2.1"

DEFAULT_FREQ_HZ = 433_200_000     # 红方广播源；蓝方是 433_920_000
DEFAULT_UDP_IP = "127.0.0.1"
DEFAULT_UDP_PORT = 14346          # 与解码器的 UDP_PORT 一致
DEFAULT_RPC_PORT = 8081           # 解码器切阵营频点用

SAMPLE_RATE_HZ = 1_000_000        # 官方 V2：1 Msps
SPS = 47                          # 官方 V2：每符号 47 采样，不能改
SENSITIVITY = 1.5628              # 官方 V2：信息波 Sensitivity
GAIN_MU = 0.175                   # M&M 时钟环增益
FREQ_ERROR = 0.0048               # 频偏预补偿；置 0 会让环路更难锁

# 射频档位。默认 balanced —— 这是我们赛场实测能正常接收的配置。
#
# full_band 是按官方常量重新算过的版本：信息波符号率 21.28kHz、峰值频偏
# 248.7kHz，调制指数 h=23.4 属于宽带 FSK，Carson 带宽是 ±270kHz。也就是说
# 沿用已久的 260kHz cutoff 比信号本身还窄 10kHz，一直在削自己的边带。
# 远距离/弱信号解不出来时，把 PROFILE 改成 "full_band" 试试。
PROFILE = "balanced"

PROFILES = {
    "balanced": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "dc_blocker_length": 32,
    },
    "full_band": {
        "rf_bandwidth_hz": 700_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 275_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "dc_blocker_length": 32,
    },
    "baseline": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
        "fir_window": "hamming",
        "dc_blocker_length": 0,
    },
    "strong": {  # 近距离/信号很强、担心前端削顶时用
        "rf_bandwidth_hz": 540_000,
        "rx_gain_db": 25.0,
        "fir_cutoff_hz": 255_000.0,
        "fir_transition_hz": 15_000.0,
        "fir_window": "blackman_harris",
        "dc_blocker_length": 32,
    },
}
# =============================================================================


WINDOWS = {
    "hamming": firdes.WIN_HAMMING,
    "blackman_harris": firdes.WIN_BLACKMAN_hARRIS,
}


class _QuietXMLRPCRequestHandler(SimpleXMLRPCRequestHandler):
    """
    吞掉每次 RPC 的 'POST /RPC2 HTTP/1.1 200 -' 访问日志。

    解码器会定期调 RPC，默认的 HTTP 处理器把每次请求都打到 stderr，
    几分钟就能刷满一屏 200，真正有用的统计行反而被冲掉，
    看上去就像程序卡在那里反复重连。
    """

    def log_message(self, format, *args):  # noqa: A002 - 覆写标准库签名
        pass


class InfoRxFlow(gr.top_block):
    """
    纯接收流图：与 tx_radio2_flow.py 的解调链逐块一致。

    这里刻意不继承 Qt.QWidget —— 界面是可选的，由外面决定要不要包一层窗口，
    这样没装 PyQt5 的机器也能直接用这个类跑无界面模式。
    """

    def __init__(
        self,
        *,
        pluto_uri: str,
        target_freq: int,
        samp_rate: int,
        rf_bandwidth: int,
        rx_gain: float,
        target_sens: float,
        fir_cutoff: float,
        fir_transition: float,
        fir_window: str,
        dc_blocker_length: int,
        udp_ip: str,
        udp_port: int,
        rpc_port: int,
        gain_mu: float,
        freq_error: float,
    ) -> None:
        gr.top_block.__init__(self, "info_rx_szu")

        self.pluto_uri = str(pluto_uri)
        self.target_freq = int(target_freq)
        self.samp_rate = int(samp_rate)
        self.rf_bandwidth = int(rf_bandwidth)
        self.rx_gain = float(rx_gain)
        self.target_sens = float(target_sens)
        self.fir_cutoff = float(fir_cutoff)
        self.fir_transition = float(fir_transition)
        self.fir_window = str(fir_window)
        self.dc_blocker_length = int(dc_blocker_length)

        # 解码器靠这个 RPC 切阵营频点；顺带允许现场热调增益/带宽。
        self.xmlrpc_server_0 = SimpleXMLRPCServer(
            ("localhost", int(rpc_port)),
            requestHandler=_QuietXMLRPCRequestHandler,
            allow_none=True,
        )
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_thread = threading.Thread(
            target=self.xmlrpc_server_0.serve_forever, daemon=True
        )
        self.xmlrpc_thread.start()

        self.iio_pluto_source_0 = iio.pluto_source(
            self.pluto_uri,
            self.target_freq,
            self.samp_rate,
            self.rf_bandwidth,
            32768,
            True,
            True,
            True,
            "manual",
            self.rx_gain,
            "",
            True,
        )

        self.dc_blocker_0 = None
        if self.dc_blocker_length > 0:
            self.dc_blocker_0 = filter.dc_blocker_cc(self.dc_blocker_length, True)

        self.low_pass_filter_0 = filter.fir_filter_ccf(1, self._make_filter_taps())
        self.digital_gfsk_demod_0 = digital.gfsk_demod(
            samples_per_symbol=SPS,
            sensitivity=self.target_sens,
            gain_mu=float(gain_mu),
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=float(freq_error),
            verbose=False,
            log=False,
        )
        # True = 一有数据就发，不等攒满；解码器那头是非阻塞收，配合得上。
        self.blocks_udp_sink_0 = blocks.udp_sink(
            gr.sizeof_char, str(udp_ip), int(udp_port), 1472, True
        )

        if self.dc_blocker_0 is not None:
            self.connect(
                (self.iio_pluto_source_0, 0),
                (self.dc_blocker_0, 0),
                (self.low_pass_filter_0, 0),
            )
        else:
            self.connect((self.iio_pluto_source_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.digital_gfsk_demod_0, 0))
        self.connect((self.digital_gfsk_demod_0, 0), (self.blocks_udp_sink_0, 0))

        self.freq_sink = None  # 有界面时由 attach_gui() 补上

    # ---- 滤波器 ----

    def _window_constant(self):
        if self.fir_window not in WINDOWS:
            raise ValueError(
                f"未知 FIR window={self.fir_window!r}，可选 {sorted(WINDOWS)}"
            )
        return WINDOWS[self.fir_window]

    def _make_filter_taps(self):
        return firdes.low_pass(
            1.0,
            float(self.samp_rate),
            float(self.fir_cutoff),
            float(self.fir_transition),
            self._window_constant(),
            6.76,
        )

    # ---- XMLRPC 接口（解码器会调用）----

    def _apply_source_params(self):
        self.iio_pluto_source_0.set_params(
            self.target_freq,
            self.samp_rate,
            self.rf_bandwidth,
            True,
            True,
            True,
            "manual",
            self.rx_gain,
            "",
            True,
        )

    def get_target_sens(self):
        return self.target_sens

    def set_target_sens(self, target_sens):
        sens = float(target_sens)
        if sens == 0.0:
            return
        self.target_sens = sens
        demod = self.digital_gfsk_demod_0
        if hasattr(demod, "fmdemod"):
            demod.fmdemod.set_gain(1.0 / sens)
        elif hasattr(demod, "set_sensitivity"):
            demod.set_sensitivity(sens)

    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = int(target_freq)
        self._apply_source_params()

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = float(rx_gain)
        self._apply_source_params()

    def get_rf_bandwidth(self):
        return self.rf_bandwidth

    def set_rf_bandwidth(self, rf_bandwidth):
        self.rf_bandwidth = int(rf_bandwidth)
        self._apply_source_params()

    def get_fir_cutoff(self):
        return self.fir_cutoff

    def set_fir_cutoff(self, fir_cutoff):
        self.fir_cutoff = float(fir_cutoff)
        self.low_pass_filter_0.set_taps(self._make_filter_taps())

    def get_fir_transition(self):
        return self.fir_transition

    def set_fir_transition(self, fir_transition):
        self.fir_transition = float(fir_transition)
        self.low_pass_filter_0.set_taps(self._make_filter_taps())

    def get_samp_rate(self):
        return self.samp_rate


def _gui_available() -> bool:
    """有没有图形界面可用：PyQt5 + sip + qtgui 都在，且有显示服务。"""
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    try:
        import sip  # noqa: F401
        from PyQt5 import Qt  # noqa: F401
        from gnuradio import qtgui  # noqa: F401
    except Exception:
        return False
    return True


def _run_with_gui(tb: InfoRxFlow) -> None:
    """带频谱窗跑。两条曲线：0=Pluto 原始 IQ，1=DC/FIR 之后，用来看削顶和滤波效果。"""
    import sip
    from PyQt5 import Qt
    from gnuradio import qtgui

    qapp = Qt.QApplication(sys.argv)

    tb.freq_sink = qtgui.freq_sink_c(
        2048,
        firdes.WIN_BLACKMAN_hARRIS,
        0,
        tb.samp_rate,
        "info RX (SZU): raw / filtered",
        2,
    )
    tb.freq_sink.set_update_time(0.10)
    tb.freq_sink.set_y_axis(-140, 10)
    tb.freq_sink.set_y_label("Relative Gain", "dB")
    tb.freq_sink.enable_autoscale(False)
    tb.freq_sink.enable_grid(True)
    tb.freq_sink.set_fft_average(0.2)
    tb.freq_sink.set_line_label(0, "raw before FIR")
    tb.freq_sink.set_line_label(1, "filtered")
    tb.freq_sink.set_line_color(0, "gray")
    tb.freq_sink.set_line_color(1, "blue")

    tb.connect((tb.iio_pluto_source_0, 0), (tb.freq_sink, 0))
    tb.connect((tb.low_pass_filter_0, 0), (tb.freq_sink, 1))

    window = Qt.QWidget()
    window.setWindowTitle("info RX (SZU)")
    layout = Qt.QVBoxLayout(window)
    layout.addWidget(sip.wrapinstance(tb.freq_sink.pyqwidget(), Qt.QWidget))
    window.resize(900, 500)

    tb.start()
    window.show()

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


def _run_headless(tb: InfoRxFlow) -> None:
    """无界面跑。解调和 UDP 输出与有界面时完全一致，只是没有频谱窗。"""

    def sig_handler(sig=None, frame=None):
        tb.stop()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    try:
        tb.wait()
    except KeyboardInterrupt:
        tb.stop()
        tb.wait()


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="info_decoder_f3_SZU 的配套收音机（正常由解码器自动拉起）"
    )
    parser.add_argument("--pluto-uri", default=DEFAULT_PLUTO_URI,
                        help="Pluto 地址，USB 直连一般是 ip:192.168.2.1")
    parser.add_argument("--freq", type=int, default=DEFAULT_FREQ_HZ,
                        help="中心频率 Hz：红方 433200000 / 蓝方 433920000")
    parser.add_argument("--udp-ip", default=DEFAULT_UDP_IP)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--rpc-port", type=int, default=DEFAULT_RPC_PORT)
    parser.add_argument("--profile", default=PROFILE, choices=sorted(PROFILES),
                        help="射频档位")
    parser.add_argument("--no-gui", action="store_true",
                        help="强制无界面（没有显示器或用 ssh 时）")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    profile = dict(PROFILES[args.profile])

    use_gui = (not args.no_gui) and _gui_available()

    print("=" * 72)
    print(f"信息波 tx_radio_szu | 档位={args.profile} | 界面={'开' if use_gui else '关'}")
    print(
        f"Pluto={args.pluto_uri} | 频点={args.freq/1e6:.3f} MHz | "
        f"RF BW={profile['rf_bandwidth_hz']/1e3:.0f} kHz | "
        f"gain={profile['rx_gain_db']:.1f} dB"
    )
    print(
        f"FIR={profile['fir_cutoff_hz']/1e3:.0f}/"
        f"{profile['fir_transition_hz']/1e3:.0f} kHz "
        f"{profile['fir_window']} | DC={profile['dc_blocker_length']}"
    )
    print(
        f"SPS={SPS} | Sens={SENSITIVITY} | "
        f"裸比特→UDP {args.udp_ip}:{args.udp_port} | RPC={args.rpc_port}"
    )
    print("若原始频谱削顶：先降 gain；数字 FIR 无法恢复已经削顶的 IQ。")
    print("=" * 72, flush=True)

    try:
        tb = InfoRxFlow(
            pluto_uri=args.pluto_uri,
            target_freq=args.freq,
            samp_rate=SAMPLE_RATE_HZ,
            rf_bandwidth=profile["rf_bandwidth_hz"],
            rx_gain=profile["rx_gain_db"],
            target_sens=SENSITIVITY,
            fir_cutoff=profile["fir_cutoff_hz"],
            fir_transition=profile["fir_transition_hz"],
            fir_window=profile["fir_window"],
            dc_blocker_length=profile["dc_blocker_length"],
            udp_ip=args.udp_ip,
            udp_port=args.udp_port,
            rpc_port=args.rpc_port,
            gain_mu=GAIN_MU,
            freq_error=FREQ_ERROR,
        )
    except Exception as exc:
        # 最常见的就是 Pluto 没插好 / 地址不对，这里给一句能直接照做的提示。
        print(f"[错误] 流图创建失败：{exc}")
        print(f"[提示] 先确认 SDR 能连上： iio_info -u {args.pluto_uri}")
        print("[提示] 地址不对就改 tx_radio_szu.py 的 DEFAULT_PLUTO_URI，")
        print("       或改 info_decoder_f3_SZU.py 的 PLUTO_URI（解码器会传给本程序）。")
        raise SystemExit(1)

    if use_gui:
        _run_with_gui(tb)
    else:
        _run_headless(tb)


if __name__ == "__main__":
    main()
