#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# 信息波【接收端】GRC —— 正常双收时绑定 SDR-B
#   Pluto URI : ip:192.168.3.1
#   XMLRPC    : localhost:8081
#   UDP Sink  : 127.0.0.1:14346 → info_decoder_f1.py
#
# 交叉测信息波 TX 时：本脚本继续用 SDR-B 收；
#   发射端用 competition/info_mock（占用 SDR-A 192.168.2.1 / RPC 8082 / UDP 12345）
#
# SPDX-License-Identifier: GPL-3.0
# GNU Radio version: v3.8.5.0-6-g57bd109d

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
import iio
try:
    from xmlrpc.server import SimpleXMLRPCServer
except ImportError:
    from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading

from gnuradio import qtgui

class tx_radio(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "tx_radio")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables —— 现场常改看这里
        # 对齐规则 V2.1.0：SPS=47，广播源 Sensitivity=1.5628
        # 纯 RX：已移除 Pluto Sink / GFSK Mod / UDP Source
        ##################################################
        # ★★★ RX Pluto URI（信息波接收机 = SDR-B）★★★
        # 改 IP 只改这一行；下面 iio.pluto_source 会引用它。
        # 正常双收：SDR-B = 192.168.3.1；交叉测干扰 TX 时本板会被 jamming_mock 占用，勿同时开本脚本。
        self.pluto_uri = pluto_uri = 'ip:192.168.3.1'
        self.target_sens = target_sens = 1.5628
        self.target_freq = target_freq = 433200000  # 默认红方广播；decoder 会 RPC 覆盖
        # 信息波弱信号；近场交叉测若仍 Header=0，可临时提到 50（注意勿削波）
        self.rx_gain = rx_gain = 45
        self.samp_rate = samp_rate = 1000000

        ##################################################
        # Blocks
        ##################################################
        # XMLRPC 端口 8081（与 info_decoder_f1.RPC_URL 一致）
        self.xmlrpc_server_0 = SimpleXMLRPCServer(('localhost', 8081), allow_none=True)
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_server_0_thread = threading.Thread(target=self.xmlrpc_server_0.serve_forever)
        self.xmlrpc_server_0_thread.daemon = True
        self.xmlrpc_server_0_thread.start()
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, #size
            firdes.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "info_filtered", #name
            1
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.low_pass_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                300e3,
                50e3,
                firdes.WIN_HAMMING,
                6.76))
        # Pluto Source：URI = 上面 Variables 里的 pluto_uri（SDR-B）
        # 增益 / LO 由 rx_gain、target_freq 与 XMLRPC 控制；URI 运行时改不了，需改变量后重启
        self.iio_pluto_source_0 = iio.pluto_source(
            pluto_uri, target_freq, 1000000, 1000000, 32768,
            True, True, True, 'manual', rx_gain, '', True)
        self.digital_gfsk_demod_0 = digital.gfsk_demod(
            samples_per_symbol=47,   # 新规则 SPS
            sensitivity=target_sens, # 1.5628
            gain_mu=0.175,           # 弱信号放软时钟环
            mu=0.5,
            omega_relative_limit=0.005,
            freq_error=0.0048,
            verbose=False,
            log=False)
        # 解调 bit 流 → decoder：UDP 14346
        self.blocks_udp_sink_0 = blocks.udp_sink(gr.sizeof_char*1, '127.0.0.1', 14346, 1472, True)


        ##################################################
        # Connections（纯 RX）
        # Source → LPF → GFSK Demod → UDP:14346
        #              ↘ Frequency Sink
        ##################################################
        self.connect((self.digital_gfsk_demod_0, 0), (self.blocks_udp_sink_0, 0))
        self.connect((self.iio_pluto_source_0, 0), (self.low_pass_filter_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.digital_gfsk_demod_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.qtgui_freq_sink_x_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "tx_radio")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def _apply_demod_sensitivity(self, sensitivity):
        """
        GR 3.8.5：gfsk_demod 构造用 quadrature_demod_cf(1.0 / sensitivity)，
        运行时 set_gain 同步写 1/sens。信息波红蓝 Sens 同为 1.5628，切阵营几乎无感。
        """
        demod = self.digital_gfsk_demod_0
        sens = float(sensitivity)
        if sens == 0.0:
            return
        gain = 1.0 / sens
        if hasattr(demod, 'fmdemod'):
            demod.fmdemod.set_gain(gain)
        elif hasattr(demod, 'set_sensitivity'):
            demod.set_sensitivity(sens)

    def get_target_sens(self):
        return self.target_sens

    def set_target_sens(self, target_sens):
        self.target_sens = float(target_sens)
        self._apply_demod_sensitivity(self.target_sens)

    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.iio_pluto_source_0.set_params(
            self.target_freq, 1000000, 1000000, True, True, True,
            'manual', self.rx_gain, '', True)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.set_target_freq(self.target_freq)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, 300e3, 50e3, firdes.WIN_HAMMING, 6.76))
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)





def main(top_block_cls=tx_radio, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

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

if __name__ == '__main__':
    main()
