#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tx_radio2 的 GNU Radio 主图；现场参数由 tx_radio2.py 传入。"""

from __future__ import annotations

import threading
from distutils.version import StrictVersion

from PyQt5 import Qt
from gnuradio import blocks, digital, filter, gr, qtgui
from gnuradio.filter import firdes
import iio
import sip

try:
    from xmlrpc.server import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer
except ImportError:
    from SimpleXMLRPCServer import (
        SimpleXMLRPCRequestHandler,
        SimpleXMLRPCServer,
    )


class _QuietXMLRPCRequestHandler(SimpleXMLRPCRequestHandler):
    """吞掉每次 RPC 的 'POST /RPC2 HTTP/1.1 200 -' 访问日志。

    解码器每 2 秒就要读一次探头、必要时还会切档，默认的 HTTP 处理器会把每次
    请求都打到 stderr。几分钟就能把终端刷满一屏 200，真正有用的统计行反而
    被冲掉了，看上去就像程序卡在那里反复重连。
    """

    def log_message(self, format, *args):  # noqa: A002 - 覆写标准库签名
        pass


WINDOWS = {
    "hamming": firdes.WIN_HAMMING,
    "blackman_harris": firdes.WIN_BLACKMAN_hARRIS,
}


class tx_radio2_flow(gr.top_block, Qt.QWidget):
    """纯 RX flowgraph；保留 gfsk_demod，先做低风险的前端/滤波增强。"""

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
    ):
        gr.top_block.__init__(self, "info_rx_antijam_f3")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("info RX anti-jam f3")
        qtgui.util.check_set_qss()

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

        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)

        self.settings = Qt.QSettings("GNU Radio", "tx_radio2")
        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except Exception:
            pass

        # XMLRPC 保持 8081，与 f1/f3 兼容；同一时刻只能运行一个信息波 GRC。
        self.xmlrpc_server_0 = SimpleXMLRPCServer(
            ("localhost", int(rpc_port)),
            requestHandler=_QuietXMLRPCRequestHandler,
            allow_none=True,
        )
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_server_0_thread = threading.Thread(
            target=self.xmlrpc_server_0.serve_forever, daemon=True
        )
        self.xmlrpc_server_0_thread.start()

        # 两条频谱：0=Pluto 原始 IQ，1=DC/FIR 后 IQ。可直接观察前端削顶与滤波效果。
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            2048,
            firdes.WIN_BLACKMAN_hARRIS,
            0,
            self.samp_rate,
            "info anti-jam: raw / filtered",
            2,
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label("Relative Gain", "dB")
        self.qtgui_freq_sink_x_0.set_trigger_mode(
            qtgui.TRIG_MODE_FREE, 0.0, 0, ""
        )
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(True)
        self.qtgui_freq_sink_x_0.set_fft_average(0.2)
        self.qtgui_freq_sink_x_0.set_line_label(0, "raw before FIR")
        self.qtgui_freq_sink_x_0.set_line_label(1, "filtered")
        self.qtgui_freq_sink_x_0.set_line_color(0, "gray")
        self.qtgui_freq_sink_x_0.set_line_color(1, "blue")
        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(
            self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget
        )
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)

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
            self.dc_blocker_0 = filter.dc_blocker_cc(
                self.dc_blocker_length, True
            )

        self.low_pass_filter_0 = filter.fir_filter_ccf(
            1, self._make_filter_taps()
        )
        self.digital_gfsk_demod_0 = digital.gfsk_demod(
            samples_per_symbol=47,
            sensitivity=self.target_sens,
            gain_mu=float(gain_mu),
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=float(freq_error),
            verbose=False,
            log=False,
        )
        self.blocks_udp_sink_0 = blocks.udp_sink(
            gr.sizeof_char, str(udp_ip), int(udp_port), 1472, True
        )

        self.connect(
            (self.iio_pluto_source_0, 0), (self.qtgui_freq_sink_x_0, 0)
        )
        if self.dc_blocker_0 is not None:
            self.connect(
                (self.iio_pluto_source_0, 0),
                (self.dc_blocker_0, 0),
                (self.low_pass_filter_0, 0),
            )
        else:
            self.connect(
                (self.iio_pluto_source_0, 0), (self.low_pass_filter_0, 0)
            )
        self.connect(
            (self.low_pass_filter_0, 0), (self.qtgui_freq_sink_x_0, 1)
        )
        self.connect(
            (self.low_pass_filter_0, 0), (self.digital_gfsk_demod_0, 0)
        )
        self.connect(
            (self.digital_gfsk_demod_0, 0), (self.blocks_udp_sink_0, 0)
        )

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

    def closeEvent(self, event):
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

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

    def _apply_demod_sensitivity(self, sensitivity):
        sens = float(sensitivity)
        if sens == 0.0:
            return
        demod = self.digital_gfsk_demod_0
        if hasattr(demod, "fmdemod"):
            demod.fmdemod.set_gain(1.0 / sens)
        elif hasattr(demod, "set_sensitivity"):
            demod.set_sensitivity(sens)

    def get_target_sens(self):
        return self.target_sens

    def set_target_sens(self, target_sens):
        self.target_sens = float(target_sens)
        self._apply_demod_sensitivity(self.target_sens)

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

    def set_samp_rate(self, samp_rate):
        self.samp_rate = int(samp_rate)
        self._apply_source_params()
        self.low_pass_filter_0.set_taps(self._make_filter_taps())
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)
