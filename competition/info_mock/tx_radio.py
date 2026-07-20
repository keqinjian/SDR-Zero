#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
competition/info_mock/tx_radio.py
=================================
信息波【发射端】GNU Radio 流程图（规则手册 V2.1.0 / 协议 V2.0.0）

---------------------------------------------------------------------------
【小白总览】本脚本只做「射频调制上天线」，不组裁判帧
---------------------------------------------------------------------------

  mock_tx.py  →  UDP:12345（27 字节空口包：Access+Header+15B）
       ▼
  本文件：UDP Source → GFSK Mod → PlutoSDR Sink
       ▼
  另一台 Pluto RX：competition/info/tx_radio.py + info_decoder_f1.py

---------------------------------------------------------------------------
【本机双板交叉测试约定】
---------------------------------------------------------------------------
  正常双收：
    SDR-A 192.168.2.1 → jamming RX（XMLRPC 8080 / UDP 14348）
    SDR-B 192.168.3.1 → info    RX（XMLRPC 8081 / UDP 14346）

  测信息波一发一收（本目录）：
    本 TX 占用 SDR-A 192.168.2.1（借 jamming 那块板发）
    info RX 仍用 SDR-B 192.168.3.1
    ★ 测试时不要同时跑 competition/jamming/tx_radio.py（A 已被占用）

---------------------------------------------------------------------------
【现场常改】
---------------------------------------------------------------------------
  target_freq —— 红 433200000 / 蓝 433920000（也可用 mock_tx --camp 经 RPC 改）
  tx_atten    —— 衰减 dB，越大越弱；信息波官方约 -60 dBm，近场可先 30～50

---------------------------------------------------------------------------
【端口与 RPC】
---------------------------------------------------------------------------
  UDP Source：127.0.0.1:12345
  XMLRPC：localhost:8082（与 RX 的 8080/8081 错开；set_target_freq / set_tx_atten 有效）

---------------------------------------------------------------------------
【物理层参数（V2）】
---------------------------------------------------------------------------
  SPS=47，Sensitivity=1.5628，BT=0.35，samp_rate=1e6
"""

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except Exception:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import blocks
from gnuradio import digital
from gnuradio import gr
import sys
import signal
import iio
try:
    from xmlrpc.server import SimpleXMLRPCServer
except ImportError:
    from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading

from gnuradio import qtgui


class tx_radio(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "info_mock_tx")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("info_mock TX (V2)")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except Exception:
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

        self.settings = Qt.QSettings("GNU Radio", "info_mock_tx")
        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except Exception:
            pass

        ##################################################
        # Variables —— 现场常改：URI / 频点 / 衰减
        ##################################################
        # 交叉测试：用 SDR-A（平时 jamming RX）当信息波发射端
        self.pluto_uri = pluto_uri = 'ip:192.168.2.1'
        # 信息波 Sens 固定 1.5628（规则表 5-23）；换阵营只换频，不换 Sens
        self.target_sens = target_sens = 1.5628
        self.target_freq = target_freq = 433200000   # 默认红方；蓝方 433920000
        self.tx_atten = tx_atten = 40.0              # dB，越大越弱（信息波官方约 -60 dBm）
        self.samp_rate = samp_rate = 1000000

        ##################################################
        # Blocks
        ##################################################
        # XMLRPC 供 mock_tx.py --camp 热切换频点（不重启 GRC）
        self.xmlrpc_server_0 = SimpleXMLRPCServer(('localhost', 8082), allow_none=True)
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_server_0_thread = threading.Thread(target=self.xmlrpc_server_0.serve_forever)
        self.xmlrpc_server_0_thread.daemon = True
        self.xmlrpc_server_0_thread.start()

        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, firdes.WIN_BLACKMAN_hARRIS, 0, samp_rate, "info_tx", 1)
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)
        for i in range(1):
            self.qtgui_freq_sink_x_0.set_line_label(i, "TX")
            self.qtgui_freq_sink_x_0.set_line_width(i, 1)
            self.qtgui_freq_sink_x_0.set_line_color(i, "blue")
            self.qtgui_freq_sink_x_0.set_line_alpha(i, 1.0)
        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)

        # UDP：收 mock_tx 打来的「字节流」（每个空口包 27B，可粘包/分包，按字节流喂调制器）
        self.blocks_udp_source_0 = blocks.udp_source(
            gr.sizeof_char * 1, '127.0.0.1', 12345, 1472, True)
        # GFSK 调制：SPS/Sens 在构造时定死；运行时 set_target_sens 无效
        self.digital_gfsk_mod_0 = digital.gfsk_mod(
            samples_per_symbol=47,
            sensitivity=target_sens,
            bt=0.35,
            verbose=False,
            log=False)
        # Pluto Sink：attenuation 越大发射越弱；带宽/采样 1e6 与 RX 一致
        self.iio_pluto_sink_0 = iio.pluto_sink(
            pluto_uri, target_freq, 1000000, 1000000, 32768,
            False, tx_atten, '', True)

        ##################################################
        # Connections：字节 → 调制 IQ → 天线上空 + 频谱监视
        ##################################################
        self.connect((self.blocks_udp_source_0, 0), (self.digital_gfsk_mod_0, 0))
        self.connect((self.digital_gfsk_mod_0, 0), (self.iio_pluto_sink_0, 0))
        self.connect((self.digital_gfsk_mod_0, 0), (self.qtgui_freq_sink_x_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "info_mock_tx")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_target_sens(self):
        return self.target_sens

    def set_target_sens(self, target_sens):
        # gfsk_mod 无运行时改 sens API；改了只记变量。改 Sens 请重启本脚本。
        self.target_sens = float(target_sens)

    def get_target_freq(self):
        return self.target_freq

    def set_target_freq(self, target_freq):
        self.target_freq = target_freq
        self.iio_pluto_sink_0.set_params(
            self.target_freq, 1000000, 1000000, self.tx_atten, '', True)

    def get_tx_atten(self):
        return self.tx_atten

    def set_tx_atten(self, tx_atten):
        self.tx_atten = float(tx_atten)
        self.set_target_freq(self.target_freq)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
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
