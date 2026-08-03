#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
info_decoder_f3_SZU.py —— 信息波抗干扰解码器（无 ROS2 · UDP 原始帧输出版）
================================================================================

【这个文件是什么】
它就是 info_decoder_f3.py，解码原理与逻辑**逐行保持不变**，只换了两头：

    进：不变，仍然从 UDP 收解调出来的 0/1 比特流
    出：不再发 ROS2 话题，改成把**校验通过的裁判系统原始帧**用 UDP 打出去，
        格式与端口与对方 epy_block 里 `self.sock.sendto(bytes(rm_frame), ...)`
        完全一致，下游程序一行都不用改。

【与对方 epy_block 的对应关系】
对方那份是嵌在 GNU Radio 流图里的 Python 块，一个进程里做完所有事：

    Pluto → 流图解调 → [epy_block: 找AC → Header → 拼MAC → CRC → 解析 → 打印]
                                                       └─ UDP:10001 发原始帧

本程序把解码搬到流图外面，接口那一头保持一模一样：

    Pluto → 流图解调 ─UDP:14346(裸比特)→ [本程序: 同样的解码链]
                                                       └─ UDP:10001 发原始帧

【怎么在对方机器上替换】
1) 流图里删掉那个 epy_block，换成一个 UDP Sink：
       blocks.udp_sink(gr.sizeof_char, "127.0.0.1", 14346, 1472, True)
   接在原来喂给 epy_block 的那根比特线上（每字节 1 个 bit，和原来一样）。
2) 跑本程序：
       python3 info_decoder_f3_SZU.py
   不需要 ROS2，不需要 numpy，不需要环境变量，只用 Python 标准库。
3) 原来监听 10001 的下游程序原样跑着，不用动。

【本程序比对方那份多做了什么】（这些正是 f3 的价值，都保留了）
1) Access 容错：64 位暗号最多错 N 位仍算命中，且**正反相都搜**；
   对方那份只搜正相，解调极性一翻就整场收不到。
2) 窗口复扫：一封业务信常跨多个 15B 空口片，片边界错位时“严格字节河”
   会暂时拼不出，但最近几片里其实已经躺着完整好帧，复扫能把它捞出来。
3) 去重：strict 和 window 两条路可能捞到同一帧，按 (seq, cmd_id, CRC16)
   去重后再发，保证 UDP 那头看到的节奏和对方原版一致（原版只有一条路，
   不会重复，所以这一步是**必须**的，否则就不是无缝替换）。
4) 长度门闩：CRC16 只有 16 位，窗口复扫要试很多偏移，撞上碰撞不是不可能；
   已知 cmd_id 的 data 长度对不上就丢掉。
5) 启动离线自检：不接 SDR 也能验证拼包/CRC/极性/窗口逻辑有没有写错。

【调参改哪里】
只改下面那块"调参面板"，全部是代码常量。
"""

from __future__ import annotations

from collections import deque
import datetime
import os
import signal
import socket
import struct
import subprocess
import sys
import time
import xmlrpc.client
from typing import Optional


# =============================================================================
# ★★★★★ 调参面板：现场优先只改这里 ★★★★★
# =============================================================================

# ---- 输入：从哪儿收解调出来的 0/1 ----
# 要与流图里 UDP Sink 的目标地址一致（每个字节只装 1 个 bit）。
UDP_IP = "127.0.0.1"
UDP_PORT = 14346

# ---- 输出：把校验通过的原始帧发到哪儿 ----
# 这两行必须与对方 epy_block 的 udp_ip / udp_port 参数一致，默认就是他们的默认值。
OUT_UDP_IP = "127.0.0.1"
OUT_UDP_PORT = 10001

# 是否转发本程序不认识的 cmd_id。
# True  = 与对方原版行为一致：只要 CRC16 过就往外发，不挑 cmd_id（推荐，最"无缝"）
# False = 只发 0x0A01~0x0A05，其它计入 unknown_cmd 丢弃
FORWARD_UNKNOWN_CMD = True

# 已知 cmd_id 的 data 长度对不上就丢弃（防 CRC16 碰撞）。
# 想要和对方原版**完全**一致（他们不做这个检查）就改 False。
ENFORCE_CMD_DATA_LEN = True

# ---- 终端显示 ----
# True = 复刻对方那份的 0.5s 战场快照界面，操作手看到的画面一模一样
SHOW_MATCH_SNAPSHOT = True
SNAPSHOT_INTERVAL_S = 0.5

# ---- 解码参数（与 info_decoder_f3.py 完全相同）----
MY_CAMP = "RED"  # 己方颜色，决定听哪个基座的广播频点

# Access 容错：2 = 64 位暗号最多错 2 位仍算找到包头；Header 仍必须完全正确。
# 参考：对方那份用的是 12。真正的防线是 Header + CRC8 + CRC16，
# Access 卡太严只会白白牺牲弱信号灵敏度；远距离收不到时可以放到 8~12。
ENABLE_FULL_ACCESS_HAMMING = True
ACCESS_MAX_HAMMING = 2

# 窗口复扫：在最近几片 15B 拼盘里再找 CRC 通过的裁判帧
ENABLE_PACKET_WINDOW_RECOVERY = True
PACKET_WINDOW_PAYLOADS = 8       # 8×15B，够盖住最长 0x0A05 及边界
FRAME_DEDUP_TTL_S = 2.0          # 同一帧 2 秒内不重复往外发

# ---- 看门狗 / 遥控（对方用自己的流图，所以默认全关）----
# 只有当对方也用我们的 tx_radio2.py 时才把这两个打开。
ENABLE_GRC_WATCHDOG = False
GRC_SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "tx_radio2.py"
)
ENABLE_XMLRPC_TUNE = False        # 是否通过 XMLRPC 让流图切到己方频点
RPC_URL = "http://127.0.0.1:8081"
UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.0

# ---- 杂项 ----
RECORD_STREAM = False            # 把 0/1 存成文本便于赛后复盘（长时间跑会很大）
RECORD_PREFIX = "info_f3_szu_record"
SELF_TEST_ON_START = True        # 启动先用假数据自检协议逻辑
STAT_INTERVAL_S = 5.0            # 统计日志间隔（秒）
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 50_000          # 0/1 缓冲上限，防内存涨爆
SERIAL_BUFFER_MAX = 8_192        # 拼好的字节河上限
# =============================================================================


# =============================================================================
# 一、空口 / 协议常量（与 info_decoder_f1.py 逐字段一致，内联进来是为了不依赖 ROS2）
# =============================================================================

INFO_FREQ_MAP = {
    "RED": 433_200_000,   # 红方广播源在红方基座
    "BLUE": 433_920_000,  # 蓝方广播源在蓝方基座
}
INFO_SENSITIVITY = 1.5628

# 信息波 64 位"暗号"，用来对齐包头（干扰波是另一串，本文件不管干扰）
ACCESS_CODE_HEX = "2F6F4C74B914492E"
AC_NORMAL = bin(int(ACCESS_CODE_HEX, 16))[2:].zfill(64)
AC_INVERTED = "".join("1" if b == "0" else "0" for b in AC_NORMAL)
AC_NORMAL_INT = int(AC_NORMAL, 2)
ACCESS_MASK = (1 << 64) - 1

AIR_ACCESS_LEN = 8
AIR_HEADER_LEN = 4
AIR_PAYLOAD_LEN = 15
AIR_FRAME_LEN = AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN  # 27 字节
AIR_FRAME_BITS = AIR_FRAME_LEN * 8                                 # 216 bit

# 官方空口 Header：两遍"后面 Payload 长度 = 15"
HEADER_OFFICIAL = bytes([0x00, 0x0F, 0x00, 0x0F])

LEN_0A05 = 41  # 协议 V2.0.0：0x0A05 data 固定 41 字节

CMD_DATA_LEN = {
    0x0A01: 24,
    0x0A02: 12,
    0x0A03: 10,
    0x0A04: 8,
    0x0A05: LEN_0A05,
}

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
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
]


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
    """'01001101…' → 字节。MSB-first：每 8 个字符里左边是高位。"""
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
    """把裁判字节流按官方空口规则切片：每 15 字节前加 Access+Header，再转比特。"""
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
# 二、日志：顶掉 ROS2 的 get_logger()
# =============================================================================
# 保持 `node.get_logger().info(...)` 这个写法不变，是为了让下面所有解码代码
# 与 info_decoder_f3.py 逐行对得上，将来两边同步改动时不容易漏。

class _Logger:
    def __init__(self, name: str) -> None:
        self.name = name

    def _emit(self, level: str, msg: str) -> None:
        stamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{level}] [{stamp}] [{self.name}]: {msg}", flush=True)

    def info(self, msg: str) -> None:
        self._emit("INFO", msg)

    def warn(self, msg: str) -> None:
        self._emit("WARN", msg)

    warning = warn

    def error(self, msg: str) -> None:
        self._emit("ERROR", msg)


# =============================================================================
# 三、战场快照：复刻对方 epy_block 的终端界面
# =============================================================================
# 这一段纯粹是"给操作手看的"，不参与任何解码判决。字段解释和排版都照抄对方
# 那份，目的是替换之后操作手看到的画面不变。
#
# ⚠ 注意 0x0A02 的字段顺序两边理解不同：
#     对方：robot_hp[0..4] = 英雄/工程/步兵3/步兵4/哨兵，[5] = 基地
#     我们 f1/f3：第 5 个 uint16 是保留位，第 6 个才是哨兵
#   这里按**对方的理解**显示以保证画面一致；但转发出去的是原始帧，
#   下游怎么解由下游自己决定，不受这里影响。建议赛前用已知血量核对一次。

ROBOT_NAMES = ["Hero", "Engineer", "Infantry3", "Infantry4", "Sentry"]

ROBOT_STATE_MAP = {
    0: "Alive",
    1: "Dead",
    2: "Invincible",
    3: "Invincible + Weak",
}


def _fresh_match_state() -> dict:
    return {
        "radar_x": [0] * 12,
        "robot_hp": [0] * 6,
        "ammo": [0] * 5,
        "remain_gold": 0,
        "total_gold": 0,
        "economy_buff": 0,
        "hp_rec": [0] * 5,
        "cooling": [0] * 5,
        "def": [0] * 5,
        "neg_def": [0] * 5,
        "atk": [0] * 5,
        "sentry_posture": 0,
        "robot_state": [0] * 5,
        "ascii_msg": "",
    }


def update_match_state(state: dict, cmd_id: int, payload: bytes) -> int:
    """
    按 cmd_id 更新快照，返回该 cmd 对应的 fresh_mask 位（没有就返回 0）。

    解析方式与对方 epy_block 的 parse_rm_payload 完全一致（含小端字节序）。
    """
    if cmd_id == 0x0A01:
        for i in range(min(len(payload) // 2, 12)):
            state["radar_x"][i] = (payload[i * 2 + 1] << 8) | payload[i * 2]
        return 1 << 0

    if cmd_id == 0x0A02:
        for i in range(min(len(payload) // 2, 6)):
            state["robot_hp"][i] = (payload[i * 2 + 1] << 8) | payload[i * 2]
        return 1 << 1

    if cmd_id == 0x0A03:
        for i in range(min(len(payload) // 2, 5)):
            state["ammo"][i] = (payload[i * 2 + 1] << 8) | payload[i * 2]
        return 1 << 2

    if cmd_id == 0x0A04:
        if len(payload) >= 8:
            state["remain_gold"] = (payload[1] << 8) | payload[0]
            state["total_gold"] = (payload[3] << 8) | payload[2]
            state["economy_buff"] = (
                (payload[7] << 24) | (payload[6] << 16)
                | (payload[5] << 8) | payload[4]
            )
        return 1 << 3

    if cmd_id == 0x0A05:
        # 0~34：5 台机器人 Buff，每台 7 字节（英雄/工程/步兵3/步兵4/哨兵）
        for i in range(5):
            o = i * 7
            if o + 6 < len(payload):
                state["hp_rec"][i] = payload[o]
                state["cooling"][i] = payload[o + 1] | (payload[o + 2] << 8)
                state["def"][i] = payload[o + 3]
                state["neg_def"][i] = payload[o + 4]
                state["atk"][i] = payload[o + 5] | (payload[o + 6] << 8)
        if len(payload) >= 36:
            state["sentry_posture"] = payload[35]
        if len(payload) >= 41:
            for i in range(5):
                state["robot_state"][i] = payload[36 + i]
        return 1 << 4

    if cmd_id == 0x0A06:
        msg_len = min(len(payload), 19)
        state["ascii_msg"] = "".join(
            chr(x) if 32 <= x <= 126 else "." for x in payload[:msg_len]
        )
        return 0

    return 0


def print_match_snapshot(state: dict, counter: int) -> None:
    print("\r\n=======================================================")
    print(f"[SZU-f3] MATCH STATE SNAPSHOT #{counter} ({SNAPSHOT_INTERVAL_S}s Refresh)")
    print("=======================================================")
    print(
        f" [Economy] Gold: {state['remain_gold']} / {state['total_gold']} | "
        f"Buff: 0x{state['economy_buff']:08X}"
    )
    print(f" [Radar]   {' '.join(str(x) for x in state['radar_x'])}")
    if state["ascii_msg"]:
        print(f" [Message] {state['ascii_msg']}")

    print("\r\n [Robots]  HP\tAmmo\tCool\tAtk\tDef\tHP_Rec\tState")
    for i in range(5):
        code = state["robot_state"][i]
        text = ROBOT_STATE_MAP.get(code, f"Unknown({code})")
        print(
            f"  - {ROBOT_NAMES[i]:9s}: "
            f"{state['robot_hp'][i]}\t"
            f"{state['ammo'][i]}\t"
            f"{state['cooling'][i]}\t"
            f"{state['atk'][i]}%\t"
            f"{state['def'][i]}%\t"
            f"{state['hp_rec'][i]}%\t"
            f"{code} ({text})"
        )
    print(f"  - Base     : {state['robot_hp'][5]}")
    print(f"\r\n [Sentry] Posture: {state['sentry_posture']}")
    print("=======================================================\r\n", flush=True)


# =============================================================================
# 四、解码器主体（这一段与 info_decoder_f3.py 逐行等价）
# =============================================================================

class InfoDecoder:
    """
    顶掉 ROS2 Node 的壳子。

    保留了 f3 里 node 用到的全部属性（ally_camp / grc_rpc / _flush_rx_buffers），
    新增一个对外 UDP 发送口，其它解码函数拿到的 node 和 f3 里一模一样。
    """

    def __init__(self) -> None:
        self._logger = _Logger("info_decoder_f3_SZU")
        self.ally_camp = MY_CAMP
        self.grc_rpc: Optional[xmlrpc.client.ServerProxy] = None
        self._flush_rx_buffers = False

        self.sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out_addr = (OUT_UDP_IP, OUT_UDP_PORT)
        self.out_frames = 0
        self.out_bytes = 0
        # 自检期间置 True：仍然走完整条判决链、仍然计数，但**不往网线上发**。
        # 否则自检用的假坐标(100,200)会在启动瞬间被灌进下游系统。
        self.emit_muted = False

        self.match_state = _fresh_match_state()
        self.fresh_mask = 0
        self.snapshot_counter = 0

        self.get_logger().info(
            f"信息波 f3-SZU 启动 | 阵营={self.ally_camp} | "
            f"Access=64bit/Hamming≤{ACCESS_MAX_HAMMING} | "
            f"窗口恢复={'开' if ENABLE_PACKET_WINDOW_RECOVERY else '关'}"
        )

    def get_logger(self) -> _Logger:
        return self._logger

    def emit_frame(self, frame: bytes) -> None:
        """
        把校验通过的裁判帧原样发出去 —— 这就是与对方 epy_block 对齐的那个接口。

        对方原版：self.sock.sendto(bytes(rm_frame), (self.udp_ip, self.udp_port))
        内容完全一致：A5 | len | seq | CRC8 | cmd_id | data | CRC16，一帧一个数据报。
        网络异常不能让解码进程死掉，所以这里吞掉异常，只记一次日志。
        """
        self.out_frames += 1
        self.out_bytes += len(frame)
        if self.emit_muted:
            return
        try:
            self.sock_out.sendto(frame, self.out_addr)
        except OSError as exc:
            self.get_logger().error(f"UDP 转发失败（已忽略）: {exc}")

    def apply_camp_to_radio(self) -> bool:
        """只有对方也用我们的 tx_radio2.py 时才有意义，默认关闭。"""
        if self.grc_rpc is None:
            return False
        freq = INFO_FREQ_MAP[self.ally_camp]
        try:
            self.grc_rpc.set_target_freq(freq)
            self.grc_rpc.set_target_sens(INFO_SENSITIVITY)
        except Exception as exc:
            self.get_logger().error(f"切频/设 Sens 失败: {exc}")
            return False
        self.get_logger().info(f"流图已切至 {freq/1e6:.3f} MHz")
        return True


def _fresh_stats() -> dict:
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "access_hamming": {0: 0, 1: 0, 2: 0},
        "header_ok": 0,
        "header_fail": 0,
        "crc8_fail": 0,
        "crc16_fail": 0,
        "frames_ok": 0,
        "strict_frames": 0,
        "window_frames": 0,
        "dedup_frames": 0,
        "unknown_cmd": 0,
        "len_mismatch": 0,
        "last_polarity": "n/a",
        "udp_bytes": 0,
    }


def _find_access(bit_buffer: str) -> Optional[tuple[int, bool, int]]:
    """
    在 0/1 海里找 Access Code（包头暗号）。

    返回 (起始下标, 是否整包反相, 汉明距离)。
    白话：拿一把 64 齿的梳子从左往右梳；允许最多差 ACCESS_MAX_HAMMING 齿。
    若正相差很多、反相却很近，说明接收极性反了，后面要把整包 0/1 翻转回来。
    """
    if len(bit_buffer) < 64:
        return None

    if not ENABLE_FULL_ACCESS_HAMMING:
        idx_n = bit_buffer.find(AC_NORMAL)
        idx_i = bit_buffer.find(AC_INVERTED)
        if idx_n < 0 and idx_i < 0:
            return None
        if idx_n >= 0 and (idx_i < 0 or idx_n <= idx_i):
            return idx_n, False, 0
        return idx_i, True, 0

    window = int(bit_buffer[:64], 2)
    last_start = len(bit_buffer) - 64
    for offset in range(last_start + 1):
        normal_distance = (window ^ AC_NORMAL_INT).bit_count()
        if normal_distance <= ACCESS_MAX_HAMMING:
            return offset, False, normal_distance
        inverted_distance = 64 - normal_distance
        if inverted_distance <= ACCESS_MAX_HAMMING:
            return offset, True, inverted_distance
        if offset < last_start:
            window = ((window << 1) & ACCESS_MASK) | (bit_buffer[offset + 64] == "1")
    return None


def extract_air_payloads(bit_buffer: str, stats: dict) -> tuple[str, list[bytes]]:
    """
    从比特流抽出空口 Payload（每片正好 15 字节）。

    步骤（像拆快递）：
      1) 用 Access 找到信封起点（可容错几位）
      2) 若反相则先翻回正相
      3) Header 必须精确是 00 0F 00 0F，否则当作假对齐，只前进 1 bit 再找
      4) 真信封才一次吃掉完整 216 bit，取出中间 15 字节
    返回：(吃剩的比特缓冲, 新得到的 Payload 列表)
    """
    payloads: list[bytes] = []
    while len(bit_buffer) >= AIR_FRAME_BITS:
        found = _find_access(bit_buffer)
        if found is None:
            bit_buffer = bit_buffer[-63:]
            break

        start, inverted, distance = found
        if len(bit_buffer) < start + AIR_FRAME_BITS:
            bit_buffer = bit_buffer[start:]
            break

        raw_bits = bit_buffer[start:start + AIR_FRAME_BITS]
        frame_bits = invert_bits(raw_bits) if inverted else raw_bits
        frame_bytes = bits_to_bytes(frame_bits)
        stats["ac_hits"] += 1
        stats["access_hamming"].setdefault(distance, 0)
        stats["access_hamming"][distance] += 1
        if inverted:
            stats["ac_hits_inverted"] += 1

        header = frame_bytes[AIR_ACCESS_LEN:AIR_ACCESS_LEN + AIR_HEADER_LEN]
        if header == HEADER_OFFICIAL:
            payloads.append(
                frame_bytes[
                    AIR_ACCESS_LEN + AIR_HEADER_LEN:
                    AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN
                ]
            )
            stats["header_ok"] += 1
            stats["last_polarity"] = "inverted" if inverted else "normal"
            bit_buffer = bit_buffer[start + AIR_FRAME_BITS:]
        else:
            stats["header_fail"] += 1
            bit_buffer = bit_buffer[start + 1:]
    return bit_buffer, payloads


def _candidate_frame(data: bytes, start: int) -> Optional[bytes]:
    """
    尝试把 data[start:] 解释成一帧裁判串口包。

    裁判帧长相（小端）：
      A5 | len(2B) | seq | CRC8 | cmd_id(2B) | data… | CRC16(2B)
    CRC8 只保护前 4 字节；CRC16 保护整帧（不含最后 2 字节自身）。
    任一环节不对就返回 None——窗口扫描靠它"试探"，不会发布半吊子帧。
    """
    if start + 9 > len(data) or data[start] != 0xA5:
        return None
    if calc_crc8(data[start:start + 4]) != data[start + 4]:
        return None
    data_len = struct.unpack_from("<H", data, start + 1)[0]
    frame_len = 5 + 2 + data_len + 2
    if data_len > 64 or frame_len > 80 or start + frame_len > len(data):
        return None
    frame = data[start:start + frame_len]
    got_crc16 = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    return frame if calc_crc16(frame[:-2]) == got_crc16 else None


def find_valid_frames(data: bytes) -> list[bytes]:
    """
    窗口复扫：不破坏原字节串，在任意位置找以 0xA5 开头且 CRC 全过的裁判帧。

    为什么需要？信息波一封业务信往往跨多个 15B 空口片；片边界一错，
    "严格按顺序吃字节"可能暂时拼不出，但窗口里其实已经躺着完整好帧。
    """
    frames: list[bytes] = []
    cursor = 0
    while cursor + 9 <= len(data):
        sof = data.find(b"\xA5", cursor)
        if sof < 0:
            break
        frame = _candidate_frame(data, sof)
        if frame is not None:
            frames.append(frame)
            cursor = sof + len(frame)
        else:
            cursor = sof + 1
    return frames


def drain_strict_frames(serial_buffer: bytearray, stats: dict) -> list[bytes]:
    """
    严格字节河：Payload 按到达顺序首尾相接，像流水线上的传送带。

    见到 0xA5 尝试读一帧；CRC8/CRC16 任一失败就丢掉 1 字节再试。
    只有校验全过才从缓冲里切除并返回——这是正式转发的主路径。
    """
    frames: list[bytes] = []
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
        if data_len > 64 or frame_len > 80:
            del serial_buffer[0]
            continue
        if len(serial_buffer) < frame_len:
            break
        frame = bytes(serial_buffer[:frame_len])
        got_crc16 = struct.unpack_from("<H", frame, frame_len - 2)[0]
        if calc_crc16(frame[:-2]) != got_crc16:
            del serial_buffer[0]
            stats["crc16_fail"] += 1
            continue
        del serial_buffer[:frame_len]
        frames.append(frame)
    return frames


def _cleanup_dedupe(dedupe: dict, now: float) -> None:
    expired = [key for key, seen in dedupe.items() if now - seen > FRAME_DEDUP_TTL_S]
    for key in expired:
        del dedupe[key]


def handle_valid_frame(
    frame: bytes,
    source: str,
    node: InfoDecoder,
    packet_counter: dict,
    stats: dict,
    dedupe: dict,
    now: float,
) -> bool:
    """
    统一处理一帧已通过 CRC 的裁判数据：去重 → 长度门闩 → UDP 转发 → 更新快照。

    与 f3 的差别只有最后一步：f3 是 parser(payload, node) 发 ROS 话题，
    这里改成 node.emit_frame(frame) 把**原始帧**发出去。判决逻辑一个字没动。

    source 只是日志标记（strict / window），不会改变"必须 CRC 通过"的铁律。
    返回 True 表示这是第一次看到并成功转发了该帧。
    """
    data_len = struct.unpack_from("<H", frame, 1)[0]
    seq = frame[3]
    cmd_id = struct.unpack_from("<H", frame, 5)[0]
    crc16 = struct.unpack_from("<H", frame, len(frame) - 2)[0]

    # 去重：strict 与 window 两条路可能捞到同一帧。对方原版只有一条路，
    # 不去重就会比原版多发，那就不叫无缝替换了。
    key = (seq, cmd_id, crc16)
    _cleanup_dedupe(dedupe, now)
    if key in dedupe:
        stats["dedup_frames"] += 1
        return False
    dedupe[key] = now

    expected = CMD_DATA_LEN.get(cmd_id)
    if ENFORCE_CMD_DATA_LEN and expected is not None and data_len != expected:
        stats["len_mismatch"] += 1
        return False

    known = cmd_id in CMD_DATA_LEN
    if not known:
        stats["unknown_cmd"] += 1
        if not FORWARD_UNKNOWN_CMD:
            return False

    payload = frame[7:7 + data_len]
    node.emit_frame(frame)

    if SHOW_MATCH_SNAPSHOT:
        node.fresh_mask |= update_match_state(node.match_state, cmd_id, payload)

    if known:
        packet_counter[f"0x{cmd_id:04X}"] += 1
    stats["frames_ok"] += 1
    stats["strict_frames" if source == "strict" else "window_frames"] += 1
    return True


# =============================================================================
# 五、离线自检（不接 SDR 也能验证解码链）
# =============================================================================

def run_self_test(node: InfoDecoder) -> bool:
    """
    覆盖 Access 0/1/2/3 位错误、反相、坏 Header、窗口扫描、去重、原始帧保真。

    全程静音：判决链照走、计数照记，但一个字节都不会发到 OUT_UDP_PORT。
    """
    node.get_logger().info("==== f3-SZU 离线自检开始 ====")
    # 自检要走完整条链（含转发判决），但假数据绝不能真的发到下游。
    node.emit_muted = True
    try:
        return _run_self_test_body(node)
    finally:
        node.emit_muted = False
        node.match_state = _fresh_match_state()
        node.fresh_mask = 0


def _run_self_test_body(node: InfoDecoder) -> bool:
    payload = struct.pack("<12H", 100, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    frame = build_referee_frame(0x0A01, payload, seq=7)
    clean_bits = wrap_stream_as_air_bits(frame)

    def corrupt_access(bits: str, positions: tuple[int, ...]) -> str:
        changed = list(bits)
        for pos in positions:
            changed[pos] = "1" if changed[pos] == "0" else "0"
        return "".join(changed)

    ok = True
    for errors, positions in ((0, ()), (1, (3,)), (2, (3, 41))):
        stats = _fresh_stats()
        remain, payloads = extract_air_payloads(
            corrupt_access(clean_bits, positions), stats
        )
        passed = len(payloads) == 3 and not remain
        ok = ok and passed
        node.get_logger().info(
            f"自检 Access {errors}bit 错误：{'通过' if passed else '失败'}"
        )

    stats = _fresh_stats()
    _, payloads_3 = extract_air_payloads(corrupt_access(clean_bits, (3, 20, 41)), stats)
    # 仅第一空口包的 Access 被破坏；后两包仍应挂上。
    reject_3 = len(payloads_3) == 2
    ok = ok and reject_3
    node.get_logger().info(f"自检 Access 3bit 拒绝：{'通过' if reject_3 else '失败'}")

    stats = _fresh_stats()
    _, inverted_payloads = extract_air_payloads(invert_bits(clean_bits), stats)
    inv_ok = bool(inverted_payloads) and stats["ac_hits_inverted"] >= 1
    ok = ok and inv_ok
    node.get_logger().info(f"自检反相：{'通过' if inv_ok else '失败'}")

    bad_header = list(clean_bits)
    bad_header[64] = "1" if bad_header[64] == "0" else "0"
    stats = _fresh_stats()
    _, bad_payloads = extract_air_payloads("".join(bad_header), stats)
    # 第一包 Header 坏，后两包仍应通过，证明失败只滑 1 bit 而不连坐。
    header_ok = len(bad_payloads) == 2 and stats["header_fail"] >= 1
    ok = ok and header_ok
    node.get_logger().info(f"自检坏 Header 拒绝：{'通过' if header_ok else '失败'}")

    window_bytes = b"".join(inverted_payloads)
    window_frames = find_valid_frames(window_bytes)
    window_ok = any(
        struct.unpack_from("<H", item, 5)[0] == 0x0A01 for item in window_frames
    )
    ok = ok and window_ok
    node.get_logger().info(f"自检包窗口 CRC 扫描：{'通过' if window_ok else '失败'}")

    if window_frames:
        test_counter = {
            key: 0 for key in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")
        }
        test_stats = _fresh_stats()
        test_dedupe: dict = {}
        before = node.out_frames
        first = handle_valid_frame(
            window_frames[0], "window", node, test_counter, test_stats,
            test_dedupe, time.time()
        )
        second = handle_valid_frame(
            window_frames[0], "strict", node, test_counter, test_stats,
            test_dedupe, time.time()
        )
        dedupe_ok = first and not second and test_stats["dedup_frames"] == 1
        # 去重生效时，对外必须只多发了 1 帧——这是"无缝替换"的核心保证。
        emit_ok = (node.out_frames - before) == 1
    else:
        dedupe_ok = False
        emit_ok = False
    ok = ok and dedupe_ok and emit_ok
    node.get_logger().info(f"自检 strict/window 去重：{'通过' if dedupe_ok else '失败'}")
    node.get_logger().info(f"自检单帧转发计数：{'通过' if emit_ok else '失败'}")

    # 防回归：自检假数据（坐标 100,200）绝不能真的打到下游端口上。
    mute_ok = node.emit_muted
    ok = ok and mute_ok
    node.get_logger().info(f"自检期间静音发送：{'通过' if mute_ok else '失败'}")

    # 转发出去的必须是原封不动的原始帧，下游才能照旧解析。
    roundtrip = build_referee_frame(0x0A02, struct.pack("<6H", 1, 2, 3, 4, 5, 6), seq=9)
    rebuilt = find_valid_frames(roundtrip)
    frame_ok = len(rebuilt) == 1 and rebuilt[0] == roundtrip
    ok = ok and frame_ok
    node.get_logger().info(f"自检原始帧保真：{'通过' if frame_ok else '失败'}")

    node.get_logger().info(f"==== f3-SZU 自检结束：{'全部通过 OK' if ok else '有失败项'} ====")
    return ok


# =============================================================================
# 六、看门狗（默认关闭；只有对方也用我们的 tx_radio2.py 时才需要）
# =============================================================================

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


def restart_grc(node: InfoDecoder) -> None:
    global _grc_process
    if not ENABLE_GRC_WATCHDOG:
        return

    if _grc_process is not None:
        node.get_logger().warn("看门狗：关闭旧 GRC 进程…")
        _kill_grc_tree(_grc_process)
        _grc_process = None

    if not os.path.isfile(GRC_SCRIPT_PATH):
        node.get_logger().error(
            f"GRC 脚本不存在: {GRC_SCRIPT_PATH}（路径在调参面板的 GRC_SCRIPT_PATH 里改）"
        )
        return

    node.get_logger().info(f"看门狗：启动 GRC → {GRC_SCRIPT_PATH}")
    env = os.environ.copy()
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
    _grc_process = subprocess.Popen(
        [sys.executable, GRC_SCRIPT_PATH], env=env, start_new_session=True
    )
    time.sleep(GRC_BOOT_WAIT_S)
    connect_grc(node)


def connect_grc(node: InfoDecoder) -> None:
    if not ENABLE_XMLRPC_TUNE:
        return
    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
        else:
            node.get_logger().error("XMLRPC 调用失败，请确认流图已启动")
    except Exception as exc:
        node.get_logger().error(f"无法连接 XMLRPC({RPC_URL}): {exc}")


# =============================================================================
# 七、主循环
# =============================================================================

def main() -> None:
    """
    按时间顺序理解即可：
      1) 建对象 + 可选离线自检
      2) 可选拉起/连接流图
      3) 绑 UDP，不断收 0/1
      4) Access+Header → 15B Payload → 拼裁判帧 → CRC → UDP 转发原始帧
      5) 顺带做窗口复扫、战场快照与看门狗重启
    """
    node = InfoDecoder()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("离线自检未通过，请勿直接用于赛场！")

    if ENABLE_GRC_WATCHDOG:
        restart_grc(node)
    else:
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
        node.get_logger().error(f"UDP 绑定 {UDP_IP}:{UDP_PORT} 失败: {exc}")
        return

    node.get_logger().info(
        f"收比特 {UDP_IP}:{UDP_PORT} → 转发原始帧 {OUT_UDP_IP}:{OUT_UDP_PORT} | "
        f"未知cmd转发={'是' if FORWARD_UNKNOWN_CMD else '否'}"
    )

    bit_buffer = ""
    serial_buffer = bytearray()
    packet_window: deque[bytes] = deque(maxlen=PACKET_WINDOW_PAYLOADS)
    dedupe: dict[tuple[int, int, int], float] = {}
    packet_counter = {
        key: 0 for key in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")
    }
    stats = _fresh_stats()
    last_stat = time.time()
    last_udp = time.time()
    last_snapshot = time.time()

    try:
        while True:
            now = time.time()
            if node._flush_rx_buffers:
                node._flush_rx_buffers = False
                bit_buffer = ""
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()
                node.get_logger().info("阵营切换：已清空全部接收缓冲")

            if ENABLE_GRC_WATCHDOG and now - last_udp > UDP_WATCHDOG_S:
                node.get_logger().warn("UDP 断流，重启流图…")
                restart_grc(node)
                last_udp = time.time()
                bit_buffer = ""
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()

            if now - last_stat >= STAT_INTERVAL_S:
                elapsed = max(0.001, now - last_stat)
                rates = " ".join(
                    f"{key}={count/elapsed:.1f}Hz"
                    for key, count in packet_counter.items()
                )
                h = stats["access_hamming"]
                node.get_logger().info(
                    f"[f3-SZU {elapsed:.1f}s] {rates} | "
                    f"AC={stats['ac_hits']} H0/1/2={h.get(0,0)}/{h.get(1,0)}/{h.get(2,0)} "
                    f"反相={stats['ac_hits_inverted']} | "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16fail={stats['crc8_fail']}/{stats['crc16_fail']} | "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"dedup={stats['dedup_frames']} 未知cmd={stats['unknown_cmd']} "
                    f"长度不符={stats['len_mismatch']} | "
                    f"收UDP={stats['udp_bytes']}B 发帧={node.out_frames}"
                )
                packet_counter = {key: 0 for key in packet_counter}
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
                    last_udp = time.time()
                    stats["udp_bytes"] += len(data)
                    incoming = "".join("1" if b else "0" for b in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]

            bit_buffer, new_payloads = extract_air_payloads(bit_buffer, stats)
            if new_payloads:
                for payload in new_payloads:
                    packet_window.append(payload)
                serial_buffer.extend(b"".join(new_payloads))
                if len(serial_buffer) > SERIAL_BUFFER_MAX:
                    del serial_buffer[:-SERIAL_BUFFER_MAX]

            for frame in drain_strict_frames(serial_buffer, stats):
                handle_valid_frame(
                    frame, "strict", node, packet_counter, stats, dedupe, now
                )

            if ENABLE_PACKET_WINDOW_RECOVERY and new_payloads:
                for frame in find_valid_frames(b"".join(packet_window)):
                    handle_valid_frame(
                        frame, "window", node, packet_counter, stats, dedupe, now
                    )

            if SHOW_MATCH_SNAPSHOT and now - last_snapshot >= SNAPSHOT_INTERVAL_S:
                node.snapshot_counter += 1
                if node.fresh_mask == 0x1F:
                    print("\r\n[+] Perfect! All 5 types of data updated within "
                          f"{SNAPSHOT_INTERVAL_S}s.")
                else:
                    missing = (~node.fresh_mask) & 0x1F
                    print(f"\r\n[-] Defect snapshot! Missing mask: 0x{missing:02X}")
                print_match_snapshot(node.match_state, node.snapshot_counter)
                node.fresh_mask = 0
                last_snapshot = now

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
        node.sock_out.close()


if __name__ == "__main__":
    main()
