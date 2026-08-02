#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
info_decoder_f1.py
==================
RoboMaster 2026 雷达站 · 信息波解码器
对齐：通信协议 V2.0.0 + 规则手册 V2.1.0

---------------------------------------------------------------------------
本程序在整条链路中的位置
---------------------------------------------------------------------------

  官方发射源(雷达基座)
       │  433 MHz 附近的 GFSK 无线电波（很弱，约 -60 dBm）
       ▼
  PlutoSDR 天线接收 → GNU Radio 里 GFSK Demod 解调
       │  解调结果：一串 0/1（每个 UDP 字节里放一个 bit）
       ▼
  ★ 本程序 ★
       │  ① 在 0/1 海里找到“空口包”（Access Code + Header + 15 字节数据）
       │  ② 把多个 15 字节片拼成完整“裁判系统串口帧”
       │  ③ CRC 校验通过后，按 cmd_id 解析坐标/血量/增益等
       ▼
  ROS2 话题 radio/info/*  （给雷达站上层软件用）

---------------------------------------------------------------------------
为什么信息波比干扰波难
---------------------------------------------------------------------------

  干扰波密钥帧 0x0A06 = 恰好 15 字节 = 1 个空口包就能装下。
  信息波一轮业务 ≈ 140 字节（新版），要拆成多个空口包连续寄出。
  中间丢 1 片，整帧 CRC 就会失败 —— 所以本程序对“假同步”非常严格。

---------------------------------------------------------------------------
相对 auto3 的修复
---------------------------------------------------------------------------

  1. 极性双搜：解调器有时把 0/1 整体反了，正反 Access Code 都搜。
  2. Header 门闩：只有 Header=00 0F 00 0F 才把 Payload 拼进长流。
  3. Header 失败时只前进 1 bit（旧逻辑会跳过整包，可能漏掉真包）。
  4. 0x0A05 按新版 41 字节解析（强化姿态 + 主要状态）。
  5. 启动时离线自检：不连无线电也能验证拼包/CRC/解析是否写对。

依赖：信息波 GRC 已设 SPS=47、Sensitivity≈1.5628，UDP → 14346；ROS2。
"""

from __future__ import annotations

import datetime
import json
import os
import signal
import socket
import struct
import subprocess
import time
import xmlrpc.client
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

# =============================================================================
# 一、运行配置（现场一般只改这几项）
# =============================================================================

# 默认己方颜色。规则：听「己方基座」上的广播源，才能收到「对方」情报。
# 运行中可用 ROS 话题 /team 覆盖：Int8 0=红，1=蓝。
MY_CAMP = "RED"

# 必须与信息波 GRC 里 UDP Sink / XMLRPC 一致
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_URL = "http://127.0.0.1:8081"

# --- 看门狗（与 jamming_decoder_f2 同款）---------------------------------
# True：由本程序拉起信息波 GRC，并在 UDP 断流时自动重启（自愈）。
# 用 mock 基带直接往 UDP 灌数据做离线调试时，务必改成 False。
# 注意 f3/f4/f5/f6 会在导入后覆盖这两个值，指向各自的 tx_radio*.py。
ENABLE_GRC_WATCHDOG = True
GRC_SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "tx_radio.py"
)
# UDP 断流多久判定 GRC 掉线并重启（秒）
UDP_WATCHDOG_S = 2.0
# GRC 启动后等它初始化 PlutoSDR 的时间（秒）
GRC_BOOT_WAIT_S = 3.0

# 把解调出的 0/1 存成文本，赛后可用 replayer 复盘
RECORD_STREAM = True
RECORD_PREFIX = "info_record"

# 启动时先跑一遍“假数据自检”（不需要 SDR）。测试机会少时强烈建议保持 True。
SELF_TEST_ON_START = True

# 统计日志间隔（秒）
STAT_INTERVAL_S = 5.0

# 没有 UDP 数据时略微休眠，避免空转占满 CPU
IDLE_SLEEP_S = 0.001

# 缓冲上限：防止长时间解不出时内存涨爆
BIT_BUFFER_MAX = 50_000      # 约数万个 bit
SERIAL_BUFFER_MAX = 8_192    # 拼包字节池

# =============================================================================
# 二、空口常量（小白版名词解释）
# =============================================================================
#
# Access Code（接入码）
#   官方规定的 64 bit“暗号”。收到的比特流里先找它，才知道一包从哪开始。
#   信息波：0x2F6F4C74B914492E ；干扰波用另一串（本文件不管干扰）。
#
# Header（包头）
#   4 字节，内容是两遍“后面 Payload 长度=15”：00 0F 00 0F（大端）。
#   作用：防止“比特碰巧长得像 Access Code”的假同步。
#
# Payload（载荷）
#   固定 15 字节有效数据。裁判系统长帧会被切成多片 Payload 连续发送。
#
# 极性（polarity）
#   GFSK 用频率高低表示 0/1。解调偶发整体反相：所有 0↔1。
#   表现：正相 Access Code 找不到，但反相版能找到。本程序两种都搜。
#

INFO_FREQ_MAP = {
    "RED": 433_200_000,   # 红方广播源 433.2 MHz（在红方基座）
    "BLUE": 433_920_000,  # 蓝方广播源 433.92 MHz（在蓝方基座）
}

# 新版规则表 5-23：广播源 Sensitivity（应与 GRC 一致）
INFO_SENSITIVITY = 1.5628

ACCESS_CODE_HEX = "2F6F4C74B914492E"
AC_NORMAL = bin(int(ACCESS_CODE_HEX, 16))[2:].zfill(64)
AC_INVERTED = "".join("1" if b == "0" else "0" for b in AC_NORMAL)

# 弱信号时 Access 前几 bit 易花：用后 48 bit 粗定位，再用 Header + 前缀复核精确定位
FUZZY_LEN = 48
FUZZY_SKIP = 64 - FUZZY_LEN  # = 16
FUZZY_NORMAL = AC_NORMAL[FUZZY_SKIP:]
FUZZY_INVERTED = AC_INVERTED[FUZZY_SKIP:]

AIR_ACCESS_LEN = 8
AIR_HEADER_LEN = 4
AIR_PAYLOAD_LEN = 15
AIR_FRAME_LEN = AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN  # 27 字节
AIR_FRAME_BITS = AIR_FRAME_LEN * 8  # 216 bit

# 官方空口 Header（只认大端正确写法，降低假阳性）
HEADER_OFFICIAL = bytes([0x00, 0x0F, 0x00, 0x0F])

# 协议 V2.0.0：0x0A05 data 固定 41 字节（不再兼容旧版 36）
LEN_0A05 = 41

# 各 cmd 的 data 段标准长度（CRC 过后再做一次长度门闩，防脏帧）
CMD_DATA_LEN = {
    0x0A01: 24,
    0x0A02: 12,
    0x0A03: 10,
    0x0A04: 8,
    0x0A05: LEN_0A05,
}

# =============================================================================
# 三、CRC：裁判系统用来查“这包有没有传错”
# =============================================================================
# 可以把它理解成快递单上的校验码：内容对得上才拆。
# CRC8 查帧头 4 字节；CRC16 查整帧（除最后 2 字节校验本身）。

CRC8_TAB = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]

CRC16_TAB = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
]

SENTRY_POSTURE_NAME = {
    1: "attack",
    2: "defense",
    3: "move",
    4: "attack_enhanced",    # 新规则
    5: "defense_enhanced",   # 新规则
    6: "move_enhanced",      # 新规则
}

ROBOT_MAIN_STATUS_NAME = {
    0: "alive",
    1: "dead",
    2: "invincible",
    3: "invincible_weak",
}


def calc_crc8(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc = CRC8_TAB[crc ^ byte]
    return crc


def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TAB[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


def bits_to_bytes(bit_string: str) -> bytes:
    """
    把 '01001101...' 这种比特字符串打包成字节。
    采用 MSB-first：每组 8 个字符里，左边是高位（与开源仓库 msb_all 一致）。
    """
    n = len(bit_string) - (len(bit_string) % 8)
    if n <= 0:
        return b""
    try:
        return bytes(int(bit_string[i:i + 8], 2) for i in range(0, n, 8))
    except ValueError:
        return b""


def bytes_to_bits(data: bytes) -> str:
    """自检用：字节 → MSB-first 比特字符串。"""
    return "".join(f"{b:08b}" for b in data)


def invert_bits(bit_string: str) -> str:
    """整体 0↔1，用于反相解调。"""
    return "".join("1" if b == "0" else "0" for b in bit_string)


def hamming(a: str, b: str) -> int:
    """两串等长比特有多少位不同。"""
    return sum(x != y for x, y in zip(a, b))


def cm_to_m(v: int) -> float:
    return v / 100.0


def build_referee_frame(cmd_id: int, payload: bytes, seq: int = 0) -> bytes:
    """
    组装一帧标准裁判系统串口包（自检用）。
    布局：A5 | len(2,小端) | seq | CRC8 | cmd_id(2) | data | CRC16(2)
    """
    header = bytearray(5)
    header[0] = 0xA5
    struct.pack_into("<H", header, 1, len(payload))
    header[3] = seq & 0xFF
    header[4] = calc_crc8(header[:4])

    body = header + struct.pack("<H", cmd_id) + payload
    frame = bytearray(body + b"\x00\x00")
    struct.pack_into("<H", frame, len(frame) - 2, calc_crc16(frame[:-2]))
    return bytes(frame)


def wrap_stream_as_air_bits(byte_stream: bytes) -> str:
    """
    把裁判字节流按官方空口规则切片：每 15 字节前加 Access+Header，再转成比特。
    长度不足 15 的倍数时右侧补 0（仅自检）。
    """
    pad = (-len(byte_stream)) % AIR_PAYLOAD_LEN
    if pad:
        byte_stream = byte_stream + bytes(pad)

    bits = []
    for i in range(0, len(byte_stream), AIR_PAYLOAD_LEN):
        packet = (
            bytes.fromhex(ACCESS_CODE_HEX)
            + HEADER_OFFICIAL
            + byte_stream[i:i + AIR_PAYLOAD_LEN]
        )
        bits.append(bytes_to_bits(packet))
    return "".join(bits)


# =============================================================================
# 四、ROS2 节点
# =============================================================================

class InfoDecoderNode(Node):
    """
    对外接口与 info_decoder_auto3 对齐，方便上层直接替换启动脚本：
      订阅 /team
      发布 radio/info/{position,hp,ammo,macro,buff}
    """

    def __init__(self) -> None:
        super().__init__("info_decoder_f1")
        self.ally_camp = MY_CAMP
        self.grc_rpc: Optional[xmlrpc.client.ServerProxy] = None
        self._flush_rx_buffers = False

        self.pub_pos = self.create_publisher(String, "radio/info/position", 10)
        self.pub_hp = self.create_publisher(String, "radio/info/hp", 10)
        self.pub_ammo = self.create_publisher(String, "radio/info/ammo", 10)
        self.pub_macro = self.create_publisher(String, "radio/info/macro", 10)
        self.pub_buff = self.create_publisher(String, "radio/info/buff", 10)

        self.create_subscription(Int8, "/team", self.team_callback, 10)
        self.get_logger().info(
            f"信息波 f1 启动 | 阵营={self.ally_camp} | UDP:{UDP_PORT} | "
            f"协议V2 0x0A05={LEN_0A05}B | 自检={'开' if SELF_TEST_ON_START else '关'}"
        )

    def publish_json(self, publisher, data_dict: dict) -> None:
        msg = String()
        msg.data = json.dumps(data_dict, ensure_ascii=False)
        publisher.publish(msg)

    def team_callback(self, msg: Int8) -> None:
        if msg.data == 0:
            new_camp = "RED"
        elif msg.data == 1:
            new_camp = "BLUE"
        else:
            return
        if new_camp == self.ally_camp:
            return
        self.ally_camp = new_camp
        self.get_logger().info(f"阵营切换 → {self.ally_camp}（听己方基座广播频点）")
        self.apply_camp_to_radio()
        # 切频后旧缓冲属于另一路信号，清掉避免脏拼包
        self._flush_rx_buffers = True

    def apply_camp_to_radio(self) -> bool:
        """
        通过 XMLRPC 改 GRC 的 target_freq。
        注意：xmlrpc ServerProxy 的 hasattr 不可靠（几乎总是 True），必须 try 真实调用。
        """
        if self.grc_rpc is None:
            return False
        freq = INFO_FREQ_MAP[self.ally_camp]
        try:
            self.grc_rpc.set_target_freq(freq)
        except Exception as exc:
            self.get_logger().error(f"切频失败 set_target_freq({freq}): {exc}")
            return False

        # Sensitivity：GRC 若没暴露该 RPC，忽略即可（应在 GRC 里写死 1.5628）
        try:
            self.grc_rpc.set_target_sens(INFO_SENSITIVITY)
        except Exception:
            pass

        self.get_logger().info(f"GNU Radio 已切至 {freq / 1e6:.3f} MHz")
        return True


# =============================================================================
# 五、业务解析（协议 V2.0.0）
# =============================================================================

def parse_0x0A01(payload: bytes, node: InfoDecoderNode) -> None:
    """对方机器人坐标。协议单位 cm，发布时转为米，方便导航。"""
    if len(payload) < 24:
        return
    c = struct.unpack("<12H", payload[:24])
    node.publish_json(
        node.pub_pos,
        {
            "hero": {"x": cm_to_m(c[0]), "y": cm_to_m(c[1])},
            "eng": {"x": cm_to_m(c[2]), "y": cm_to_m(c[3])},
            "inf3": {"x": cm_to_m(c[4]), "y": cm_to_m(c[5])},
            "inf4": {"x": cm_to_m(c[6]), "y": cm_to_m(c[7])},
            "aerial": {"x": cm_to_m(c[8]), "y": cm_to_m(c[9])},
            "sentry": {"x": cm_to_m(c[10]), "y": cm_to_m(c[11])},
            "unit": "m",
        },
    )


def parse_0x0A02(payload: bytes, node: InfoDecoderNode) -> None:
    """对方血量。第 5 个 uint16 为保留位。"""
    if len(payload) < 12:
        return
    hero, eng, inf3, inf4, _reserved, sentry = struct.unpack("<6H", payload[:12])
    node.publish_json(
        node.pub_hp,
        {"hero": hero, "eng": eng, "inf3": inf3, "inf4": inf4, "sentry": sentry},
    )


def parse_0x0A03(payload: bytes, node: InfoDecoderNode) -> None:
    """对方允许发弹量。"""
    if len(payload) < 10:
        return
    hero, inf3, inf4, aerial, sentry = struct.unpack("<5H", payload[:10])
    node.publish_json(
        node.pub_ammo,
        {"hero": hero, "inf3": inf3, "inf4": inf4, "aerial": aerial, "sentry": sentry},
    )


def parse_0x0A04(payload: bytes, node: InfoDecoderNode) -> None:
    """
    对方金币 + 占领/交互状态位图（协议表 1-47，uint32 小端）。
    bit0 补给区；bit1-2 中央高地；bit3 梯形高地；bit4-5 堡垒；
    bit6-7 前哨站增益；bit8 基地增益；bit9-15 地形跨越/场地交互模块。
    """
    if len(payload) < 8:
        return
    coin, total_coin, status = struct.unpack("<HHI", payload[:8])
    node.publish_json(
        node.pub_macro,
        {
            "coin": coin,
            "total_coin": total_coin,
            "occupation": {
                "supply": status & 0x01,
                "center_highland": (status >> 1) & 0x03,
                "trapezoid_highland": (status >> 3) & 0x01,
                "fortress": (status >> 4) & 0x03,
                "outpost_buff": (status >> 6) & 0x03,
                "base_buff": (status >> 8) & 0x01,
                # bit9~15：飞坡隧道 / 高地 / 公路等场地交互模块（1=检测到对方）
                "enemy_tunnel_pre": (status >> 9) & 0x01,
                "enemy_tunnel_post": (status >> 10) & 0x01,
                "ally_tunnel_pre": (status >> 11) & 0x01,
                "ally_tunnel_post": (status >> 12) & 0x01,
                "enemy_highland_top": (status >> 13) & 0x01,
                "enemy_fly_ramp_rear": (status >> 14) & 0x01,
                "enemy_highway_top": (status >> 15) & 0x01,
                "raw_bits": status,
            },
        },
    )


def parse_0x0A05(payload: bytes, node: InfoDecoderNode) -> None:
    """
    对方增益 / 哨兵姿态 / 主要状态（协议 V2：固定 41 字节）。

      字节 0~34 : 5 单位 × 7B（回血% / 冷却 / 防御% / 负防御% / 攻击%）
      字节 35   : 哨兵姿态 1~6
      字节 36~40: 英雄/工程/步兵3/步兵4/哨兵 主要状态 0~3
    """
    if len(payload) < LEN_0A05:
        return

    robot_keys = ["hero", "eng", "inf3", "inf4", "sentry"]
    units = {}
    for i, key in enumerate(robot_keys):
        heal, cool, defense, neg_def, attack = struct.unpack_from("<BHBBH", payload, i * 7)
        units[key] = {
            "heal_pct": heal,
            "cooling": cool,
            "defense_pct": defense,
            "negative_defense_pct": neg_def,
            "attack_pct": attack,
            # 兼容 auto3 旧字段名，避免上层崩
            "vulnerability": neg_def,
            "attack": attack,
        }

    posture_code = payload[35]
    main_status = {}
    for i, key in enumerate(robot_keys):
        code = payload[36 + i]
        main_status[key] = {
            "code": code,
            "name": ROBOT_MAIN_STATUS_NAME.get(code, "unknown"),
        }

    node.publish_json(
        node.pub_buff,
        {
            "units": units,
            "sentry_posture": posture_code,
            "sentry_posture_name": SENTRY_POSTURE_NAME.get(posture_code, "unknown"),
            "main_status": main_status,
        },
    )


# =============================================================================
# 六、空口同步（最容易写错的一层）
# =============================================================================

def _prefix_ok(frame_bits: str) -> bool:
    """
    模糊匹配只用了后 48 bit；这里再核对前 16 bit。
    frame_bits 必须已是“极性纠正后”的比特，因此始终与正相 AC 前缀比较。
    允许最多 2 bit 误差（弱信号花码），否则视为假同步。
    """
    return hamming(frame_bits[:FUZZY_SKIP], AC_NORMAL[:FUZZY_SKIP]) <= 2


def extract_air_payloads(bit_buffer: str, stats: dict) -> tuple[str, bytearray]:
    """
    从连续比特流中抽出通过校验的 15 字节 Payload。

    搜索策略（与开源思路对齐，并贴合你们模糊匹配习惯）：
      1) 在正相/反相模糊 Access（后 48 bit）里找最早命中；
      2) 回退 16 bit 对齐包起点；
      3) 若整帧被判为反相，先整体取反；
      4) 检查 Header == 00 0F 00 0F；
      5) 再检查 Access 前 16 bit 汉明距离 ≤ 2；
      6) 通过才把 Payload 拼进输出。

    【关键修复】Header/前缀失败时只前进 1 bit，
    绝不能跳过整整 216 bit —— 否则真包会被“连坐”丢掉。
    """
    appended = bytearray()

    while len(bit_buffer) >= AIR_FRAME_BITS:
        idx_n = bit_buffer.find(FUZZY_NORMAL)
        idx_i = bit_buffer.find(FUZZY_INVERTED)

        target_idx = -1
        match_inverted_pattern = False
        if idx_n != -1 and (idx_i == -1 or idx_n <= idx_i):
            target_idx = idx_n
            match_inverted_pattern = False
        elif idx_i != -1:
            target_idx = idx_i
            match_inverted_pattern = True

        if target_idx == -1:
            # 保留可能跨 UDP 分片的半截模糊码
            keep = FUZZY_LEN - 1
            bit_buffer = bit_buffer[-keep:] if len(bit_buffer) > keep else bit_buffer
            break

        start = target_idx - FUZZY_SKIP
        if start < 0:
            # 模糊码贴在缓冲区头，还不够回退：丢掉 1 bit 再试
            bit_buffer = bit_buffer[target_idx + 1:]
            continue
        if len(bit_buffer) < start + AIR_FRAME_BITS:
            # 包还不完整，等更多 UDP
            break

        raw_bits = bit_buffer[start:start + AIR_FRAME_BITS]
        # 若匹配到的是“反相 Access 图案”，说明解调极性反了 → 整包取反后按正相解析
        frame_bits = invert_bits(raw_bits) if match_inverted_pattern else raw_bits
        stats["ac_hits"] += 1
        if match_inverted_pattern:
            stats["ac_hits_inverted"] += 1

        frame_bytes = bits_to_bytes(frame_bits)
        accepted = False
        if len(frame_bytes) >= AIR_FRAME_LEN:
            header = frame_bytes[AIR_ACCESS_LEN:AIR_ACCESS_LEN + AIR_HEADER_LEN]
            if header == HEADER_OFFICIAL and _prefix_ok(frame_bits):
                appended.extend(
                    frame_bytes[
                        AIR_ACCESS_LEN + AIR_HEADER_LEN:
                        AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN
                    ]
                )
                stats["header_ok"] += 1
                stats["last_polarity"] = (
                    "inverted" if match_inverted_pattern else "normal"
                )
                accepted = True
            else:
                stats["header_fail"] += 1

        if accepted:
            # 真包：跳到下一包起点
            bit_buffer = bit_buffer[start + AIR_FRAME_BITS:]
        else:
            # 假同步：只前进 1 bit，避免漏掉紧挨着的真 Access
            bit_buffer = bit_buffer[start + 1:]

    return bit_buffer, appended


def drain_referee_frames(
    serial_buffer: bytearray,
    node: InfoDecoderNode,
    packet_counter: dict,
    stats: dict,
) -> None:
    """
    在“已拼接的 Payload 字节河”里捞裁判帧。

    一帧结构：
      [0]     0xA5          起始字节（SOF）
      [1:3]   data_length   后面 data 有多长（小端）
      [3]     seq           序号
      [4]     CRC8          只保护前 4 字节
      [5:7]   cmd_id        如 0x0A01
      [7:...] data
      [末2]   CRC16         保护整帧除自身外所有字节

    信息波一帧经常跨 2~4 个空口包，所以字节不够时要 break 等待，不能当失败丢掉。
    """
    while len(serial_buffer) >= 9:
        sof = serial_buffer.find(0xA5)
        if sof < 0:
            serial_buffer.clear()
            break
        if sof > 0:
            del serial_buffer[:sof]
        if len(serial_buffer) < 5:
            break

        if calc_crc8(serial_buffer[:4]) != serial_buffer[4]:
            del serial_buffer[0]
            stats["crc8_fail"] += 1
            continue

        data_len = struct.unpack_from("<H", serial_buffer, 1)[0]
        frame_len = 5 + 2 + data_len + 2

        # 新版最大帧约 50 字节；过大几乎一定是误同步
        if data_len > 64 or frame_len > 80:
            del serial_buffer[0]
            continue
        if len(serial_buffer) < frame_len:
            break  # 等更多 Payload 片

        frame = bytes(serial_buffer[:frame_len])
        got_crc16 = struct.unpack_from("<H", frame, frame_len - 2)[0]
        if calc_crc16(frame[:-2]) != got_crc16:
            del serial_buffer[0]
            stats["crc16_fail"] += 1
            continue

        cmd_id = struct.unpack_from("<H", frame, 5)[0]
        payload = frame[7:7 + data_len]
        del serial_buffer[:frame_len]
        stats["frames_ok"] += 1

        # 长度门闩：已知 cmd 必须严格匹配协议 data 长度
        expected = CMD_DATA_LEN.get(cmd_id)
        if expected is not None and data_len != expected:
            stats["len_mismatch"] += 1
            continue

        if cmd_id == 0x0A01:
            parse_0x0A01(payload, node)
            packet_counter["0x0A01"] += 1
        elif cmd_id == 0x0A02:
            parse_0x0A02(payload, node)
            packet_counter["0x0A02"] += 1
        elif cmd_id == 0x0A03:
            parse_0x0A03(payload, node)
            packet_counter["0x0A03"] += 1
        elif cmd_id == 0x0A04:
            parse_0x0A04(payload, node)
            packet_counter["0x0A04"] += 1
        elif cmd_id == 0x0A05:
            parse_0x0A05(payload, node)
            packet_counter["0x0A05"] += 1
        else:
            stats["unknown_cmd"] += 1


# =============================================================================
# 七、离线自检（不插 SDR 也能验证核心逻辑）
# =============================================================================

def run_self_test(node: InfoDecoderNode) -> bool:
    """
    造一帧 0x0A01 → 切成空口包 → 再解调链路反解出来。
    顺带测一遍反相流，确保极性双搜没写反。
    """
    node.get_logger().info("==== 离线自检开始（不需要无线电）====")
    ok = True

    # 构造可辨认的坐标：英雄 (1.00m, 2.00m) → 协议 100cm, 200cm
    payload = struct.pack("<12H", 100, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    frame = build_referee_frame(0x0A01, payload, seq=1)
    if calc_crc8(frame[:4]) != frame[4] or calc_crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        node.get_logger().error("自检失败：自己组装的帧 CRC 就不对（查 CRC 表）")
        return False

    air_bits = wrap_stream_as_air_bits(frame)

    def _feed(bits: str, label: str) -> bool:
        stats = {
            "ac_hits": 0,
            "ac_hits_inverted": 0,
            "header_ok": 0,
            "header_fail": 0,
            "crc8_fail": 0,
            "crc16_fail": 0,
            "frames_ok": 0,
            "unknown_cmd": 0,
            "len_mismatch": 0,
            "last_polarity": "n/a",
        }
        counter = {k: 0 for k in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")}
        serial = bytearray()
        remain, payloads = extract_air_payloads(bits, stats)
        serial.extend(payloads)
        drain_referee_frames(serial, node, counter, stats)
        passed = counter["0x0A01"] >= 1 and stats["header_ok"] >= 1
        node.get_logger().info(
            f"自检[{label}] {'通过' if passed else '失败'} | "
            f"0x0A01={counter['0x0A01']} HeaderOK={stats['header_ok']} "
            f"极性命中反相图案={stats['ac_hits_inverted']}"
        )
        return passed

    if not _feed(air_bits, "正相"):
        ok = False
    if not _feed(invert_bits(air_bits), "反相"):
        ok = False

    # 再测 0x0A05（41 字节）能否过长度门闩
    buff_payload = bytes([0] * LEN_0A05)
    buff_payload = bytearray(buff_payload)
    buff_payload[35] = 4  # 强化进攻
    buff_payload[36] = 0
    frame05 = build_referee_frame(0x0A05, bytes(buff_payload), seq=2)
    air05 = wrap_stream_as_air_bits(frame05)
    stats = {
        "ac_hits": 0, "ac_hits_inverted": 0, "header_ok": 0, "header_fail": 0,
        "crc8_fail": 0, "crc16_fail": 0, "frames_ok": 0, "unknown_cmd": 0,
        "len_mismatch": 0, "last_polarity": "n/a",
    }
    counter = {k: 0 for k in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")}
    serial = bytearray()
    _, payloads = extract_air_payloads(air05, stats)
    serial.extend(payloads)
    drain_referee_frames(serial, node, counter, stats)
    if counter["0x0A05"] < 1:
        node.get_logger().error("自检失败：0x0A05(41B) 未能解析")
        ok = False
    else:
        node.get_logger().info("自检[0x0A05新版] 通过")

    node.get_logger().info(
        "==== 离线自检结束：%s ====" % ("全部通过 OK" if ok else "存在失败 FAIL")
    )
    return ok


# =============================================================================
# 八、主循环
# =============================================================================

def connect_grc(node: InfoDecoderNode) -> None:
    """
    ServerProxy() 构造时不会真正连网，必须调用一次 RPC 才知道 GRC 在不在。
    仅在「不启用看门狗」时用：本程序不管 GRC，只连它的 XMLRPC。
    """
    try:
        rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        node.grc_rpc = rpc
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
        else:
            node.get_logger().error(
                f"XMLRPC 已创建但调用失败。请先启动信息波 GRC（端口 8081）"
            )
    except Exception as exc:
        node.get_logger().error(f"无法创建 XMLRPC({RPC_URL}): {exc}")


# =============================================================================
# 八(前)、GNU Radio 看门狗
# =============================================================================
#
# 为什么信息波也需要看门狗？
#   信息波是 -60 dBm 级别的弱信号，PlutoSDR / GRC 在长时间运行、USB 抖动、
#   缓冲溢出时可能「悄悄卡死」——进程还在，但不再往 UDP 吐比特。
#   看门狗做两件事：
#     1) 开局由本程序拉起 GRC（省得手动开两个终端）；
#     2) 主循环里发现「UDP 断流超过 UDP_WATCHDOG_S 秒」，就杀掉旧 GRC、
#        重新拉起并重连 XMLRPC、重新切到己方频点，实现自愈。

_grc_process: Optional[subprocess.Popen] = None


def _kill_grc_tree(proc: subprocess.Popen) -> None:
    """杀进程组，避免 Qt/GNU Radio 子进程残留占用 Pluto。"""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        try:
            os.kill(proc.pid, signal.SIGTERM)
        except Exception:
            pass
    try:
        proc.wait(timeout=2)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                os.kill(proc.pid, signal.SIGKILL)
            except Exception:
                pass


def restart_grc(node: InfoDecoderNode) -> None:
    """杀掉旧 GRC（若有）→ 重新启动 → 重连 XMLRPC → 重新切频。"""
    global _grc_process
    if not ENABLE_GRC_WATCHDOG:
        return

    if _grc_process is not None:
        node.get_logger().warn("看门狗：关闭旧信息波 GRC 进程…")
        _kill_grc_tree(_grc_process)
        _grc_process = None

    if not os.path.isfile(GRC_SCRIPT_PATH):
        node.get_logger().error(
            f"GRC 脚本不存在: {GRC_SCRIPT_PATH}"
            "（路径在各解码器调参面板的 GRC_SCRIPT_PATH 里改）"
        )
        return

    node.get_logger().info(f"看门狗：启动信息波 GRC → {GRC_SCRIPT_PATH}")
    env = os.environ.copy()
    # 有桌面保留默认 Qt（频谱窗）；仅无 DISPLAY 时才 offscreen
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
    _grc_process = subprocess.Popen(
        ["python3", GRC_SCRIPT_PATH],
        env=env,
        start_new_session=True,
    )
    time.sleep(GRC_BOOT_WAIT_S)

    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info("看门狗：XMLRPC 重连成功")
        else:
            node.get_logger().error("看门狗：XMLRPC 已创建但调用失败")
    except Exception as exc:
        node.get_logger().error(f"看门狗：XMLRPC 重连失败: {exc}")
        node.grc_rpc = None


def _fresh_stats() -> dict:
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "header_ok": 0,
        "header_fail": 0,
        "crc8_fail": 0,
        "crc16_fail": 0,
        "frames_ok": 0,
        "unknown_cmd": 0,
        "len_mismatch": 0,
        "last_polarity": "n/a",
        "udp_bytes": 0,
    }


def main() -> None:
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START:
        if not run_self_test(node):
            node.get_logger().error(
                "离线自检未通过！请先修代码再上无线电，避免浪费测试窗口。"
            )
            # 不强制退出，方便你仍想盯现场；但默认应修到通过
        else:
            node.get_logger().info("离线自检通过，开始连接无线电链路。")

    if ENABLE_GRC_WATCHDOG:
        # 看门狗模式：由本程序拉起 GRC 并负责重连
        restart_grc(node)
    else:
        # 手动模式：只连你已经跑起来的 GRC
        connect_grc(node)

    file_recorder = None
    if RECORD_STREAM:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"{RECORD_PREFIX}_{node.ally_camp}_{stamp}.txt"
        try:
            file_recorder = open(path, "w", encoding="utf-8")
            node.get_logger().info(f"比特录制: {path}")
        except OSError as exc:
            node.get_logger().error(f"录制文件打开失败: {exc}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as exc:
        node.get_logger().error(
            f"UDP 绑定 {UDP_IP}:{UDP_PORT} 失败: {exc}（是否有旧进程占用？）"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"开始监听解调比特 | {UDP_IP}:{UDP_PORT} | "
        f"请确认 GRC：SPS=47, Sens≈{INFO_SENSITIVITY}, 无 TX"
    )

    bit_buffer = ""
    serial_buffer = bytearray()
    last_stat = time.time()
    last_udp = time.time()          # 看门狗：上次收到 UDP 的时刻
    packet_counter = {k: 0 for k in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")}
    stats = _fresh_stats()

    try:
        while rclpy.ok():
            now = time.time()

            if node._flush_rx_buffers:
                node._flush_rx_buffers = False
                bit_buffer = ""
                serial_buffer.clear()
                node.get_logger().info("阵营切换：已清空 bit/serial 缓冲")

            # ---- 看门狗：UDP 断流超时则重启 GRC ----
            if ENABLE_GRC_WATCHDOG and (now - last_udp > UDP_WATCHDOG_S):
                node.get_logger().warn(
                    f"看门狗：{UDP_WATCHDOG_S:.1f}s 无 UDP 数据，重启信息波 GRC…"
                )
                restart_grc(node)
                # 必须用重启完成后的时刻；用循环开头的 now 会立刻再次触发看门狗
                last_udp = time.time()
                bit_buffer = ""
                serial_buffer.clear()

            if now - last_stat >= STAT_INTERVAL_S:
                # 读统计时的排障口诀（写在日志旁白）：
                #   UDP字节=0           → GRC 没跑 / 端口不对 / 解调无输出
                #   AC命中多 Header=0   → SPS/Sens 仍错，或滤波/增益导致误码
                #   Header有通过 CRC多失败 → 拼包被插脏片（少见，因有门闩）或丢片
                #   帧计数开始涨        → 成功
                node.get_logger().info(
                    f"[统计{STAT_INTERVAL_S:.0f}s] 业务帧={packet_counter} | "
                    f"AC={stats['ac_hits']}(反相图案{stats['ac_hits_inverted']}) "
                    f"Header OK {stats['header_ok']}/FAIL {stats['header_fail']} "
                    f"CRC8 fail {stats['crc8_fail']} CRC16 fail {stats['crc16_fail']} "
                    f"长度不符{stats['len_mismatch']} "
                    f"极性={stats['last_polarity']} UDP字节={stats['udp_bytes']}"
                )
                packet_counter = {k: 0 for k in packet_counter}
                polarity = stats["last_polarity"]
                stats = _fresh_stats()
                stats["last_polarity"] = polarity
                last_stat = now
                if file_recorder is not None:
                    file_recorder.flush()

            got_udp = False
            try:
                while True:
                    data, _ = sock.recvfrom(16384)
                    got_udp = True
                    last_udp = time.time()      # 看门狗喂狗
                    stats["udp_bytes"] += len(data)
                    # GNU Radio UDP Sink：每个字节的数值就是 0 或 1
                    incoming = "".join(str(b) for b in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]

            bit_buffer, new_payloads = extract_air_payloads(bit_buffer, stats)
            if new_payloads:
                serial_buffer.extend(new_payloads)
                if len(serial_buffer) > SERIAL_BUFFER_MAX:
                    del serial_buffer[:-SERIAL_BUFFER_MAX]

            drain_referee_frames(serial_buffer, node, packet_counter, stats)

            rclpy.spin_once(node, timeout_sec=0.0)
            if not got_udp:
                time.sleep(IDLE_SLEEP_S)

    except KeyboardInterrupt:
        node.get_logger().info("中断退出")
    finally:
        if _grc_process is not None:
            _kill_grc_tree(_grc_process)
        if file_recorder is not None:
            file_recorder.flush()
            file_recorder.close()
        sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
