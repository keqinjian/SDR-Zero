#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jamming_decoder_f2.py
=====================
RoboMaster 2026 雷达站 · 干扰波解码器（新一代）
对齐：通信协议 V2.0.0 + 规则手册 V2.1.0

---------------------------------------------------------------------------
本程序在整条链路中的位置
---------------------------------------------------------------------------

  官方干扰源（放在【己方】雷达基座上）
       │  发射 GFSK 干扰波（很强，约 -10 dBm）
       │  里面夹着「对方自定义的 6 位密钥」
       ▼
  PlutoSDR + GNU Radio(GFSK Demod, SPS=47)
       │  UDP 输出一串 0/1
       ▼
  ★ 本程序 ★
       │  找干扰波 Access Code → 校验 Header → 抽出 15 字节 Payload
       │  拼出裁判帧 0x0A06 → 读出 6 位 ASCII 密钥
       ▼
  ROS2 话题 radar/jamming_key
       │  每次 CRC 通过的 0x0A06 都发一次（同一密钥重复收到也会发）
       ▼
  （上层）通过裁判系统 0x0121 把密钥交给服务器验证

---------------------------------------------------------------------------

  「红方干扰源放在红方基座，携带蓝方自定义密钥」——蓝方同理。
  所以：己方是红 → 听红方三级干扰频点（432.2 / 432.5 / 432.8 MHz）。
  旧版 f1 用 enemy_camp 扫敌方频点，几何上是错的；本版改为 ally_camp。

---------------------------------------------------------------------------
相对 jamming_decoder_f1 的升级
---------------------------------------------------------------------------

  1. 【修旧 bug】阵营语义改为己方基座频点 + /team 切换
  2. 【修旧 bug】Header 门闩 + 前缀复核；失败只前进 1 bit
  3. 【对齐新规则】Sensitivity 表 2.8194 / 2.5681 / 0.6517
  4. 【增强】密钥按字节投票（CRC 偶发失败时仍可能恢复）
  5. 【增强】启动离线自检（正相/反相/组帧）
  6. 详细中文注释，方便无线电小白维护

依赖：干扰波 GRC 已设 SPS=47，UDP→14348，XMLRPC→8080；ROS2。
"""

from __future__ import annotations

import datetime
import os
import signal
import socket
import struct
import subprocess
import time
import xmlrpc.client
from collections import Counter
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String

# =============================================================================
# 一、运行配置（现场一般只改这里）
# =============================================================================

# 默认己方颜色；运行中 /team (0=红, 1=蓝) 可覆盖
MY_CAMP = "RED"

UDP_IP = "127.0.0.1"
UDP_PORT = 14348
RPC_URL = "http://127.0.0.1:8080"

# 环境变量 RM_ENABLE_GRC_WATCHDOG=0 可关闭（mock 基带注入时务必关）
ENABLE_GRC_WATCHDOG = os.environ.get("RM_ENABLE_GRC_WATCHDOG", "1") not in (
    "0", "false", "False", "no", "NO",
)
# 默认：与本文件同目录的 tx_radio.py；可用 JAMMING_GRC_SCRIPT 覆盖
GRC_SCRIPT_PATH = os.environ.get(
    "JAMMING_GRC_SCRIPT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "tx_radio.py"),
)

RECORD_STREAM = True
RECORD_PREFIX = "jamming_record"

SELF_TEST_ON_START = True
STAT_INTERVAL_S = 5.0
IDLE_SLEEP_S = 0.001

# 失锁后多久扫下一档干扰等级（秒）
SCAN_INTERVAL_S = 3.0
# 多久没解出密钥视为失锁（秒）
LOCK_TIMEOUT_S = 5.0
# UDP 断流多久触发 GRC 重启（秒）
UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.0

BIT_BUFFER_MAX = 50_000
SERIAL_BUFFER_MAX = 4_096

# 投票恢复：至少多少个候选、每字节最少多少票才采纳
KEY_VOTE_MIN_CANDIDATES = 3
KEY_VOTE_MIN_PER_BYTE = 2
# True：投票恢复的密钥也发到 radar/jamming_key（可能误报，默认关；只走 meta）
# 比赛里建议仅用 strict CRC 密钥做 0x0121 上报，投票结果只作调试参考。
PUBLISH_RECOVERED_KEY = False

# =============================================================================
# 二、空口 / 射频常量（小白版）
# =============================================================================
#
# Access Code（干扰波）：0x16E8D377151C712D —— 和信息波那串不同！
# Header：仍然是 00 0F 00 0F（Payload 长度=15）
# 0x0A06 整帧恰好 15 字节 = 1 个空口 Payload，所以比信息波好解得多。
#

ACCESS_CODE_HEX = "16E8D377151C712D"
AC_NORMAL = bin(int(ACCESS_CODE_HEX, 16))[2:].zfill(64)
AC_INVERTED = "".join("1" if b == "0" else "0" for b in AC_NORMAL)

FUZZY_LEN = 48
FUZZY_SKIP = 64 - FUZZY_LEN  # 16
FUZZY_NORMAL = AC_NORMAL[FUZZY_SKIP:]
FUZZY_INVERTED = AC_INVERTED[FUZZY_SKIP:]

AIR_ACCESS_LEN = 8
AIR_HEADER_LEN = 4
AIR_PAYLOAD_LEN = 15
AIR_FRAME_LEN = AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN  # 27
AIR_FRAME_BITS = AIR_FRAME_LEN * 8  # 216

HEADER_OFFICIAL = bytes([0x00, 0x0F, 0x00, 0x0F])

# 新版规则表 5-23：己方阵营 → [(freq_hz, sensitivity), ...] 对应干扰等级 1/2/3
# 扫频顺序：先一级（带宽最宽最好收）→ 二级 → 三级
JAM_FREQ_MAP = {
    "RED": [
        (432_200_000, 2.8194),  # 一级
        (432_500_000, 2.5681),  # 二级
        (432_800_000, 0.6517),  # 三级
    ],
    "BLUE": [
        (434_920_000, 2.8194),
        (434_620_000, 2.5681),
        (434_320_000, 0.6517),
    ],
}

CMD_JAM_KEY = 0x0A06
JAM_KEY_DATA_LEN = 6
# 0x0A06 整帧长度：5(头) + 2(cmd) + 6(data) + 2(crc16) = 15
JAM_FRAME_LEN = 15

# =============================================================================
# 三、CRC
# =============================================================================

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
    n = len(bit_string) - (len(bit_string) % 8)
    if n <= 0:
        return b""
    try:
        return bytes(int(bit_string[i:i + 8], 2) for i in range(0, n, 8))
    except ValueError:
        return b""


def bytes_to_bits(data: bytes) -> str:
    return "".join(f"{b:08b}" for b in data)


def invert_bits(bit_string: str) -> str:
    return "".join("1" if b == "0" else "0" for b in bit_string)


def hamming(a: str, b: str) -> int:
    return sum(x != y for x, y in zip(a, b))


def is_alnum_key(text: str) -> bool:
    """官方密钥：恰好 6 位，字母或数字。"""
    return len(text) == 6 and text.isalnum()


def sanitize_key_bytes(raw: bytes) -> Optional[str]:
    if len(raw) < 6:
        return None
    chars = []
    for b in raw[:6]:
        if 48 <= b <= 57 or 65 <= b <= 90 or 97 <= b <= 122:
            chars.append(chr(b))
        else:
            return None
    key = "".join(chars)
    return key if is_alnum_key(key) else None


def build_referee_frame(cmd_id: int, payload: bytes, seq: int = 0) -> bytes:
    header = bytearray(5)
    header[0] = 0xA5
    struct.pack_into("<H", header, 1, len(payload))
    header[3] = seq & 0xFF
    header[4] = calc_crc8(header[:4])
    body = header + struct.pack("<H", cmd_id) + payload
    frame = bytearray(body + b"\x00\x00")
    struct.pack_into("<H", frame, len(frame) - 2, calc_crc16(frame[:-2]))
    return bytes(frame)


def wrap_as_air_bits(byte_stream: bytes) -> str:
    """把裁判字节流切成空口包比特（自检用）。干扰波一帧正好 15B → 一包。"""
    pad = (-len(byte_stream)) % AIR_PAYLOAD_LEN
    if pad:
        byte_stream = byte_stream + bytes(pad)
    out = []
    for i in range(0, len(byte_stream), AIR_PAYLOAD_LEN):
        packet = (
            bytes.fromhex(ACCESS_CODE_HEX)
            + HEADER_OFFICIAL
            + byte_stream[i:i + AIR_PAYLOAD_LEN]
        )
        out.append(bytes_to_bits(packet))
    return "".join(out)


# =============================================================================
# 四、ROS2 节点
# =============================================================================

class JammingDecoderNode(Node):
    def __init__(self) -> None:
        super().__init__("jamming_decoder_f2")
        self.ally_camp = MY_CAMP
        self._camp_changed = False
        self.pub_key = self.create_publisher(String, "radar/jamming_key", 10)
        # 可选：区分严格 CRC 密钥 / 投票恢复密钥，方便上层策略
        self.pub_key_meta = self.create_publisher(String, "radar/jamming_key_meta", 10)
        self.create_subscription(Int8, "/team", self.team_callback, 10)
        self.get_logger().info(
            f"干扰波 f2 启动 | 己方阵营={self.ally_camp} | UDP:{UDP_PORT} | "
            f"新版 Sensitivity 表 | 自检={'开' if SELF_TEST_ON_START else '关'}"
        )

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
        self.get_logger().info(f"阵营切换 → {self.ally_camp}（听己方基座干扰频点）")
        # 立即切到己方一级频点，勿等主循环（与 info_decoder_f1 对齐）
        self._camp_changed = True

    def publish_key(self, key: str, source: str = "strict", *, log: bool = True) -> None:
        """
        每次成功解出密钥都发布（含同一密钥重复收到）。
        source=strict：CRC 通过 → radar/jamming_key + meta
        source=recovered：投票恢复 → 默认只 meta；PUBLISH_RECOVERED_KEY=True 才进主话题
        log：是否打 info 日志（重复密钥建议 False，避免 10Hz 刷屏）
        """
        meta = String()
        meta.data = f'{{"key":"{key}","source":"{source}"}}'
        self.pub_key_meta.publish(meta)

        if source == "strict" or PUBLISH_RECOVERED_KEY:
            msg = String()
            msg.data = key
            self.pub_key.publish(msg)

        if log:
            self.get_logger().info(f"密钥截获 [{source}]: {key}")


# =============================================================================
# 五、空口同步 + 密钥提取
# =============================================================================

def _prefix_ok(frame_bits: str) -> bool:
    """模糊匹配后复核 Access 前 16 bit（允 2 bit 花码）。frame_bits 须已极性纠正。"""
    return hamming(frame_bits[:FUZZY_SKIP], AC_NORMAL[:FUZZY_SKIP]) <= 2


def extract_air_payloads(bit_buffer: str, stats: dict) -> tuple[str, bytearray]:
    """
    与 info_decoder_f1 同策略：
      模糊 AC 双搜 → Header 门闩 → 前缀复核 → 失败只 +1 bit。
    """
    appended = bytearray()

    while len(bit_buffer) >= AIR_FRAME_BITS:
        idx_n = bit_buffer.find(FUZZY_NORMAL)
        idx_i = bit_buffer.find(FUZZY_INVERTED)

        target_idx = -1
        match_inv = False
        if idx_n != -1 and (idx_i == -1 or idx_n <= idx_i):
            target_idx, match_inv = idx_n, False
        elif idx_i != -1:
            target_idx, match_inv = idx_i, True

        if target_idx == -1:
            keep = FUZZY_LEN - 1
            bit_buffer = bit_buffer[-keep:] if len(bit_buffer) > keep else bit_buffer
            break

        start = target_idx - FUZZY_SKIP
        if start < 0:
            bit_buffer = bit_buffer[target_idx + 1:]
            continue
        if len(bit_buffer) < start + AIR_FRAME_BITS:
            break

        raw = bit_buffer[start:start + AIR_FRAME_BITS]
        frame_bits = invert_bits(raw) if match_inv else raw
        stats["ac_hits"] += 1
        if match_inv:
            stats["ac_hits_inverted"] += 1

        frame_bytes = bits_to_bytes(frame_bits)
        accepted = False
        if len(frame_bytes) >= AIR_FRAME_LEN:
            header = frame_bytes[AIR_ACCESS_LEN:AIR_ACCESS_LEN + AIR_HEADER_LEN]
            if header == HEADER_OFFICIAL and _prefix_ok(frame_bits):
                payload = frame_bytes[
                    AIR_ACCESS_LEN + AIR_HEADER_LEN:
                    AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN
                ]
                appended.extend(payload)
                stats["header_ok"] += 1
                stats["last_polarity"] = "inverted" if match_inv else "normal"
                accepted = True
            else:
                stats["header_fail"] += 1

        if accepted:
            bit_buffer = bit_buffer[start + AIR_FRAME_BITS:]
        else:
            bit_buffer = bit_buffer[start + 1:]

    return bit_buffer, appended


def looks_like_jam_payload(payload: bytes) -> bool:
    """
    未过 CRC 时，粗看 Payload 是否像 0x0A06 整帧：
      A5 | 06 00 | seq | crc8 | 06 0A | key[6] | crc16
    cmd_id 小端 = 0x0A06 → 字节 06 0A。
    """
    if len(payload) != AIR_PAYLOAD_LEN:
        return False
    if payload[0] != 0xA5:
        return False
    # data_length 应为 6
    if payload[1] != 0x06 or payload[2] != 0x00:
        return False
    # cmd_id 允许少量 bit 错（开源用汉明距离）
    cmd_hamming = (payload[5] ^ 0x06).bit_count() + (payload[6] ^ 0x0A).bit_count()
    if cmd_hamming > 4:
        return False
    key = payload[7:13]
    printable = sum(
        48 <= b <= 57 or 65 <= b <= 90 or 97 <= b <= 122 for b in key
    )
    return printable >= 4


def vote_key_from_candidates(candidates: list[bytes]) -> Optional[str]:
    """
    对多个「看起来像密钥」的 6 字节做逐字节投票。
    干扰波强但偶发误码时，可比单次 CRC 更稳（开源仓库同思路）。
    """
    if len(candidates) < KEY_VOTE_MIN_CANDIDATES:
        return None
    recovered = bytearray()
    for i in range(6):
        counts = Counter(c[i] for c in candidates if len(c) >= 6)
        printable = {
            v: n
            for v, n in counts.items()
            if 48 <= v <= 57 or 65 <= v <= 90 or 97 <= v <= 122
        }
        if not printable:
            return None
        best, votes = max(printable.items(), key=lambda kv: (kv[1], kv[0]))
        if votes < KEY_VOTE_MIN_PER_BYTE:
            return None
        recovered.append(best)
    key = recovered.decode("ascii", errors="ignore")
    return key if is_alnum_key(key) else None


def drain_jam_frames(
    serial_buffer: bytearray,
    node: JammingDecoderNode,
    stats: dict,
    vote_pool: list[bytes],
    last_key_holder: list,
) -> None:
    """
    在 Payload 字节流里找 0x0A06。
    投票候选只从「当前缓冲里整段 15 字节且像密钥帧」的片收集一次逻辑：
    在 CRC 路径之外，对缓冲按 15 字节对齐探测（干扰波一帧=一包，对齐很自然）。
    """
    # 对齐探测：从每个偏移 0..14 试一次，避免 SOF 不在片首时漏投票
    probe = bytes(serial_buffer)
    if len(probe) >= AIR_PAYLOAD_LEN:
        for align in range(min(15, len(probe) - AIR_PAYLOAD_LEN + 1)):
            chunk = probe[align:align + AIR_PAYLOAD_LEN]
            if looks_like_jam_payload(chunk):
                vote_pool.append(bytes(chunk[7:13]))
                break  # 每个 drain 周期最多收 1 个投票样本，防池子爆炸

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
        if data_len != JAM_KEY_DATA_LEN or frame_len != JAM_FRAME_LEN:
            # 干扰波业务帧几乎只有 0x0A06；长度不对就滑 1 字节
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

        cmd_id = struct.unpack_from("<H", frame, 5)[0]
        payload = frame[7:7 + data_len]
        del serial_buffer[:frame_len]

        if cmd_id != CMD_JAM_KEY:
            stats["unknown_cmd"] += 1
            continue

        key = sanitize_key_bytes(payload)
        if key is None:
            stats["bad_key"] += 1
            continue

        stats["frames_ok"] += 1
        stats["strict_keys"] += 1
        # 每次 CRC 通过的密钥都发话题（含重复）；仅日志在密钥变化时打，避免 10Hz 刷屏
        changed = key != last_key_holder[0]
        if not changed:
            stats["dup_keys"] += 1
        node.publish_key(key, source="strict", log=changed)
        last_key_holder[0] = key


# =============================================================================
# 六、GNU Radio 看门狗 / 扫频
# =============================================================================

_grc_process = None
_grc_rpc: Optional[xmlrpc.client.ServerProxy] = None


def _kill_grc_tree(proc: subprocess.Popen) -> None:
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


def restart_grc(node: JammingDecoderNode) -> None:
    global _grc_process, _grc_rpc
    if not ENABLE_GRC_WATCHDOG:
        return

    if _grc_process is not None:
        node.get_logger().warn("看门狗：关闭旧 GRC 进程…")
        _kill_grc_tree(_grc_process)
        _grc_process = None

    if not os.path.isfile(GRC_SCRIPT_PATH):
        node.get_logger().error(
            f"GRC 脚本不存在: {GRC_SCRIPT_PATH}（可用 JAMMING_GRC_SCRIPT 覆盖）"
        )
        return

    node.get_logger().info(f"看门狗：启动 GRC → {GRC_SCRIPT_PATH}")
    env = os.environ.copy()
    # 有桌面则用默认 Qt；仅无 DISPLAY 时才 offscreen，避免频谱窗打不开
    if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
    _grc_process = subprocess.Popen(
        ["python3", GRC_SCRIPT_PATH],
        env=env,
        start_new_session=True,
    )
    time.sleep(GRC_BOOT_WAIT_S)
    try:
        _grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        freqs = JAM_FREQ_MAP[node.ally_camp]
        _grc_rpc.set_target_freq(freqs[0][0])
        _grc_rpc.set_target_sens(freqs[0][1])
        node.get_logger().info("看门狗：XMLRPC 重连成功")
    except Exception as exc:
        node.get_logger().error(f"看门狗：XMLRPC 失败: {exc}")
        _grc_rpc = None


def apply_scan_channel(node: JammingDecoderNode, level_idx: int) -> None:
    """level_idx: 0/1/2 → 干扰等级 1/2/3。"""
    global _grc_rpc
    table = JAM_FREQ_MAP[node.ally_camp]
    level_idx %= len(table)
    freq, sens = table[level_idx]
    if _grc_rpc is None:
        try:
            _grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        except Exception:
            return
    try:
        _grc_rpc.set_target_freq(freq)
        _grc_rpc.set_target_sens(sens)
        node.get_logger().info(
            f"扫频 → 己方{node.ally_camp} 等级{level_idx + 1}: "
            f"{freq / 1e6:.3f} MHz  Sens={sens}"
        )
    except Exception as exc:
        node.get_logger().error(f"扫频 RPC 失败: {exc}")
        _grc_rpc = None


# =============================================================================
# 七、离线自检
# =============================================================================

def run_self_test(node: JammingDecoderNode) -> bool:
    node.get_logger().info("==== 干扰波离线自检开始 ====")
    ok = True
    sample_key = "Ab12Xy"
    frame = build_referee_frame(CMD_JAM_KEY, sample_key.encode("ascii"), seq=1)
    if len(frame) != JAM_FRAME_LEN:
        node.get_logger().error(f"自检失败：0x0A06 帧长={len(frame)}，应为 {JAM_FRAME_LEN}")
        return False

    air = wrap_as_air_bits(frame)

    def _feed(bits: str, label: str) -> bool:
        stats = _fresh_stats()
        serial = bytearray()
        vote_pool: list[bytes] = []
        last_key = [""]
        _, payloads = extract_air_payloads(bits, stats)
        serial.extend(payloads)
        drain_jam_frames(serial, node, stats, vote_pool, last_key)
        passed = last_key[0] == sample_key and stats["header_ok"] >= 1
        node.get_logger().info(
            f"自检[{label}] {'通过' if passed else '失败'} | "
            f"key={last_key[0]!r} HeaderOK={stats['header_ok']} "
            f"反相图案命中={stats['ac_hits_inverted']}"
        )
        return passed

    if not _feed(air, "正相"):
        ok = False
    if not _feed(invert_bits(air), "反相"):
        ok = False

    # 投票恢复：造 3 份略有损伤但可识别的 key 字节
    cands = [b"Ab12Xy", b"Ab12Xy", b"Ab12Xz"]
    voted = vote_key_from_candidates(cands)
    if voted != "Ab12Xy":
        node.get_logger().error(f"自检失败：投票恢复得到 {voted!r}")
        ok = False
    else:
        node.get_logger().info("自检[投票恢复] 通过")

    node.get_logger().info(
        "==== 干扰波离线自检结束：%s ====" % ("全部通过 OK" if ok else "存在失败 FAIL")
    )
    return ok


def _fresh_stats() -> dict:
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "header_ok": 0,
        "header_fail": 0,
        "crc8_fail": 0,
        "crc16_fail": 0,
        "frames_ok": 0,
        "strict_keys": 0,
        "dup_keys": 0,
        "bad_key": 0,
        "unknown_cmd": 0,
        "recovered_keys": 0,
        "last_polarity": "n/a",
        "udp_bytes": 0,
    }


# =============================================================================
# 八、主循环
# =============================================================================

def main() -> None:
    global _grc_rpc

    rclpy.init()
    node = JammingDecoderNode()

    if SELF_TEST_ON_START:
        if not run_self_test(node):
            node.get_logger().error("离线自检未通过！建议先修代码再上场。")
        else:
            node.get_logger().info("离线自检通过，进入无线电接收。")

    if ENABLE_GRC_WATCHDOG:
        restart_grc(node)
    else:
        try:
            _grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
            apply_scan_channel(node, 0)
        except Exception as exc:
            node.get_logger().error(f"连接 XMLRPC 失败: {exc}")

    file_recorder = None
    if RECORD_STREAM:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = f"{RECORD_PREFIX}_{node.ally_camp}_{stamp}.txt"
        try:
            file_recorder = open(path, "w", encoding="utf-8")
            node.get_logger().info(f"比特录制: {path}")
        except OSError as exc:
            node.get_logger().error(f"录制打开失败: {exc}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setblocking(False)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as exc:
        node.get_logger().error(f"UDP 绑定失败 {UDP_IP}:{UDP_PORT}: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"监听干扰波比特 | {UDP_IP}:{UDP_PORT} | "
        f"请确认 GRC：SPS=47，Sens 可 RPC 切换，无 TX"
    )

    bit_buffer = ""
    serial_buffer = bytearray()
    vote_pool: list[bytes] = []
    last_key_holder = [""]          # 最近一次 strict 密钥（日志变化检测用；话题每次都发）
    last_recovered_holder = [""]    # meta 投票去重，互不污染
    stats = _fresh_stats()
    last_stat = time.time()
    last_success = time.time()
    last_udp = time.time()
    last_scan = time.time()
    locked = False
    scan_idx = 0
    last_camp = node.ally_camp

    try:
        while rclpy.ok():
            now = time.time()

            if node._camp_changed or node.ally_camp != last_camp:
                node._camp_changed = False
                last_camp = node.ally_camp
                scan_idx = 0
                locked = False
                bit_buffer = ""
                serial_buffer.clear()
                vote_pool.clear()
                apply_scan_channel(node, scan_idx)
                last_scan = now
                last_success = now
                last_udp = time.time()

            if now - last_stat >= STAT_INTERVAL_S:
                node.get_logger().info(
                    f"[统计{STAT_INTERVAL_S:.0f}s] "
                    f"严格密钥={stats['strict_keys']} 重复={stats['dup_keys']} "
                    f"投票恢复={stats['recovered_keys']} | "
                    f"AC={stats['ac_hits']}(反相{stats['ac_hits_inverted']}) "
                    f"Header OK {stats['header_ok']}/FAIL {stats['header_fail']} "
                    f"CRC8 {stats['crc8_fail']} CRC16 {stats['crc16_fail']} "
                    f"锁定={'是' if locked else '否'} 等级={scan_idx + 1} "
                    f"极性={stats['last_polarity']} UDP字节={stats['udp_bytes']}"
                )
                polarity = stats["last_polarity"]
                stats = _fresh_stats()
                stats["last_polarity"] = polarity
                last_stat = now
                if file_recorder is not None:
                    file_recorder.flush()
                # 投票池太大就裁剪，保留最近
                if len(vote_pool) > 64:
                    vote_pool[:] = vote_pool[-32:]

            # UDP 看门狗
            if ENABLE_GRC_WATCHDOG and (now - last_udp > UDP_WATCHDOG_S):
                node.get_logger().error("UDP 断流，重启 GRC")
                restart_grc(node)
                apply_scan_channel(node, scan_idx)
                # 必须用重启完成后的时刻，否则会连环重启
                last_udp = time.time()
                last_success = time.time()
                bit_buffer = ""
                serial_buffer.clear()
                locked = False
                continue

            # 失锁扫频：在己方 1→2→3 级之间轮换
            n_levels = len(JAM_FREQ_MAP[node.ally_camp])
            if (not locked) and (now - last_success > SCAN_INTERVAL_S) and (now - last_scan > SCAN_INTERVAL_S):
                scan_idx = (scan_idx + 1) % n_levels
                apply_scan_channel(node, scan_idx)
                last_scan = now
                last_success = now
                bit_buffer = ""
                serial_buffer.clear()
                vote_pool.clear()

            got_udp = False
            try:
                while True:
                    data, _ = sock.recvfrom(16384)
                    got_udp = True
                    last_udp = time.time()
                    stats["udp_bytes"] += len(data)
                    incoming = "".join(str(b) for b in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]

            before_keys = stats["strict_keys"]
            bit_buffer, payloads = extract_air_payloads(bit_buffer, stats)
            if payloads:
                serial_buffer.extend(payloads)
                if len(serial_buffer) > SERIAL_BUFFER_MAX:
                    del serial_buffer[:-SERIAL_BUFFER_MAX]

            drain_jam_frames(serial_buffer, node, stats, vote_pool, last_key_holder)

            if stats["strict_keys"] > before_keys:
                locked = True
                last_success = now

            # 投票恢复：只写 meta 去重，不污染 strict 的 last_key_holder
            if (not locked) and len(vote_pool) >= KEY_VOTE_MIN_CANDIDATES:
                voted = vote_key_from_candidates(vote_pool[-16:])
                if voted and voted != last_recovered_holder[0]:
                    node.publish_key(voted, source="recovered")
                    last_recovered_holder[0] = voted
                    stats["recovered_keys"] += 1
                    if PUBLISH_RECOVERED_KEY:
                        # 允许把恢复密钥当正式结果时，才占用主话题去重与锁定
                        last_key_holder[0] = voted
                        locked = True
                        last_success = now
                    vote_pool.clear()

            if locked and (now - last_success > LOCK_TIMEOUT_S):
                locked = False
                node.get_logger().warn("锁定超时，恢复扫频")

            rclpy.spin_once(node, timeout_sec=0.0)
            if not got_udp:
                time.sleep(IDLE_SLEEP_S)

    except KeyboardInterrupt:
        node.get_logger().info("中断退出")
    finally:
        if file_recorder is not None:
            file_recorder.flush()
            file_recorder.close()
        sock.close()
        if _grc_process is not None:
            _kill_grc_tree(_grc_process)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
