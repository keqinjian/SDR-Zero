#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
competition/jamming_mock/tx_radio.py
====================================
干扰波【发射端】GNU Radio 流程图（规则手册 V2.1.0 / 协议 V2.0.0）

---------------------------------------------------------------------------
【小白总览】本脚本只做「射频调制上天线」，不组 0x0A06
---------------------------------------------------------------------------

  mock_tx.py  →  UDP:12347（空口包：干扰 Access + Header + 15B）
       ▼
  本文件：UDP Source → GFSK Mod → PlutoSDR Sink
       ▼
  另一台 Pluto RX：competition/jamming/tx_radio.py + jamming_decoder_f2.py

---------------------------------------------------------------------------
【换干扰等级（最易踩坑）】
---------------------------------------------------------------------------
  等级同时决定【频点】和【Sensitivity】：
    一级 Sens=2.8194 / 二级 2.5681 / 三级 0.6517
  gfsk_mod 构造后不能热改 Sens → 换等级必须：
    1) 改本文件 target_freq + target_sens
    2) 重启本脚本
  mock_tx.py --level 只会 RPC 改频点，并打印「Sens 应对齐」提醒。

---------------------------------------------------------------------------
【本机双板交叉测试约定】
---------------------------------------------------------------------------
  正常双收：
    SDR-A 192.168.2.1 → jamming RX（XMLRPC 8080 / UDP 14348）
    SDR-B 192.168.3.1 → info    RX（XMLRPC 8081 / UDP 14346）

  测干扰波一发一收（本目录）：
    本 TX 占用 SDR-B 192.168.3.1（借 info 那块板发）
    jamming RX 仍用 SDR-A 192.168.2.1
    ★ 测试时不要同时跑 competition/info/tx_radio.py（B 已被占用）

---------------------------------------------------------------------------
【现场常改】
---------------------------------------------------------------------------
  target_freq + target_sens（换等级必须一起改并重启）
  tx_atten：干扰约 -10 dBm，衰减宜小（默认 10）

---------------------------------------------------------------------------
【端口】UDP 12347 | XMLRPC 8083（与 RX 的 8080/8081 错开）
---------------------------------------------------------------------------
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
        gr.top_block.__init__(self, "jamming_mock_tx")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("jamming_mock TX (V2)")
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

        self.settings = Qt.QSettings("GNU Radio", "jamming_mock_tx")
        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except Exception:
            pass

        ##################################################
        # Variables —— 现场常改：URI / 等级频点+Sens / 衰减
        ##################################################
        # 交叉测试：用 SDR-B（平时 info RX）当干扰波发射端
        self.pluto_uri = pluto_uri = 'ip:192.168.3.1'
        # 红方：L1 432.2/2.8194 | L2 432.5/2.5681 | L3 432.8/0.6517
        # 蓝方：L1 434.92 / L2 434.62 / L3 434.32，Sens 与等级一一对应
        # ★ 换等级：频点与 Sens 必须一起改，并重启本脚本
        self.target_sens = target_sens = 2.8194
        self.target_freq = target_freq = 432200000
        self.tx_atten = tx_atten = 10.0              # 干扰官方约 -10 dBm，衰减宜小
        self.samp_rate = samp_rate = 1000000

        ##################################################
        # Blocks
        ##################################################
        # XMLRPC：mock_tx --camp/--level 可热切频；Sens 热切无效
        self.xmlrpc_server_0 = SimpleXMLRPCServer(('localhost', 8083), allow_none=True)
        self.xmlrpc_server_0.register_instance(self)
        self.xmlrpc_server_0_thread = threading.Thread(target=self.xmlrpc_server_0.serve_forever)
        self.xmlrpc_server_0_thread.daemon = True
        self.xmlrpc_server_0_thread.start()

        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, firdes.WIN_BLACKMAN_hARRIS, 0, samp_rate, "jamming_tx", 1)
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
            self.qtgui_freq_sink_x_0.set_line_color(i, "red")
            self.qtgui_freq_sink_x_0.set_line_alpha(i, 1.0)
        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)

        # UDP：收 mock_tx 空口字节流（Access 为干扰波专用 16E8…，勿与信息波混）
        self.blocks_udp_source_0 = blocks.udp_source(
            gr.sizeof_char * 1, '127.0.0.1', 12347, 1472, True)
        # SPS=47（V2）；Sens 必须与当前等级一致，否则 RX 解调很差
        self.digital_gfsk_mod_0 = digital.gfsk_mod(
            samples_per_symbol=47,
            sensitivity=target_sens,
            bt=0.35,
            verbose=False,
            log=False)
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
        self.settings = Qt.QSettings("GNU Radio", "jamming_mock_tx")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_target_sens(self):
        return self.target_sens

    def set_target_sens(self, target_sens):
        # gfsk_mod 运行时改 sens 需重建块；换等级请改初值后重启本脚本
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
