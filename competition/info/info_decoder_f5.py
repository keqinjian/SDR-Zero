#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
info_decoder_f5.py —— 现场自适应信息波解码器（给无线电小白的说明）
================================================================

【一句话】不用你手工来回改增益/带宽：程序看着统计数字，自动换接收档。

【相对 f4 多了什么】
  1) 物理层优先用更稳的 symbol_sync（M&M + MMSE 插值），对不齐再回退旧时钟环
  2) 每 2 秒看一眼：有没有 Access？Header？CRC 帧？UDP 忙不忙？
     然后经 XMLRPC 热切换 RF 带宽 / 增益 / FIR（见 tx_radio5_tunes.py）
  3) Access 容错会在 2～4 之间自己收紧/放宽（假同步多就收紧）
  4) 比特缓冲更长，弱信号跨很多空口片时不容易“忘了前半截”

【白话比喻】
  f3/f4 像固定档相机；f5 像带简易自动曝光：拍糊了就换一组预设参数再试。
  它仍然不会“猜”CRC 失败的业务数据——自动调的是收音机旋钮，不是裁判帧内容。

【硬约束（别改歪）】
  SPS=47、Sens=1.5628、Header 精确匹配、CRC 通过才发 ROS。

【怎么启动】
  python3 competition/info/info_decoder_f5.py
  就这一条命令，不需要任何环境变量。它会自动拉起 tx_radio5.py。

【要调参改哪里】
  - 射频档位表 + 开机拓扑 → tx_radio5_tunes.py 顶部的调参面板
  - 自动换档脾气 / 协议 / 运行方式 → 本文件下面的调参面板
  - 背景与调试流程 → competition/docs/信息波f5自适应调试指南.md
"""

from __future__ import annotations

from collections import deque
import datetime
import os
import socket
import struct
import time
import xmlrpc.client
from typing import Optional

import rclpy
from std_msgs.msg import Int8, String

import info_decoder_f1 as base
import info_decoder_f3 as f3
from tx_radio5_tunes import (
    AUTO_TUNE_ORDER,
    BOOT_PROFILE,
    RUNTIME_TUNES,
)


# =============================================================================
# ★★★★★ 调参面板：现场优先只改这里 ★★★★★
#
# 直接 `python3 competition/info/info_decoder_f5.py` 启动，不需要任何环境变量。
# 射频档位（增益 / 带宽 / FIR）不在本文件，在 tx_radio5_tunes.py。
# =============================================================================

# 我方阵营。也可以让 /team 话题在运行中改（0=红, 1=蓝），这里只是开机默认值。
MY_CAMP = "RED"

# 与 tx_radio5.py 之间的本机管道，端口冲突时才动。
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_URL = "http://127.0.0.1:8081"

# 是否允许运行中自动换 RF 档。
# 改成 False 就变成"固定档接收"，用来和自动模式做对照实验；
# 此时实际用的是 tx_radio5_tunes.BOOT_PROFILE 选中的那一档。
ENABLE_AUTO_TUNE = True

# Access Code（64bit 同步字）容错上下限，程序会在这两个值之间自己收放：
#   LOOSE 4 = 平时用，弱信号更容易抓到包头
#   TIGHT 2 = 发现"假包很多、CRC 全挂"时自动收紧
ACCESS_MAX_HAMMING_LOOSE = 4
ACCESS_MAX_HAMMING_TIGHT = 2

# Header 不允许花码，放宽必然引入假帧。除非做实验，否则不要改这个 0。
HEADER_MAX_HAMMING = 0

ENABLE_PACKET_WINDOW_RECOVERY = True
PACKET_WINDOW_PAYLOADS = 8
FRAME_DEDUP_TTL_S = 2.0

# 看门狗：True 时本程序会自动拉起 tx_radio5.py 并在 UDP 断流时重启它。
# 想手动分两个终端跑（比如要盯频谱窗），改成 False，然后自己先跑 tx_radio5.py。
ENABLE_GRC_WATCHDOG = True
GRC_SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "tx_radio5.py"
)

UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.5

# 把收到的原始 0/1 存成文本，事后复盘用。会增加磁盘 I/O。
RECORD_STREAM = False
RECORD_PREFIX = "info_f5_record"

SELF_TEST_ON_START = True
STAT_INTERVAL_S = 2.0
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 180_000  # 比 f3/f4 更长，给弱信号拼包留余量
SERIAL_BUFFER_MAX = 16_384

# ---- 自动换档"脾气"参数（单位：秒或计数，均相对 2s 统计窗）----
# 觉得它换档太频繁 → 把 COOLDOWN 和 SILENCE 调大；
# 觉得它反应太慢   → 调小，但别小于统计窗 2s，否则一窗数据还没攒够就乱切。
AUTO_COOLDOWN_S = 6.0  # 两次大换档最短间隔
AUTO_EXPLORE_SILENCE_S = 4.0  # 无帧多久开始探索
AUTO_LOCK_GOOD_WINDOWS = 3  # 连续几个好窗口就锁定当前档
AUTO_RELOCK_SILENCE_S = 10.0  # 锁定后再无帧多久重新探索
# =============================================================================

# 当前开机拓扑名，只用于日志与"是否允许自动"的判断；值在 tx_radio5_tunes.py 改。
F5_PROFILE = BOOT_PROFILE

INFO_FREQ_MAP = base.INFO_FREQ_MAP
INFO_SENSITIVITY = base.INFO_SENSITIVITY
AC_NORMAL = base.AC_NORMAL
AC_NORMAL_INT = int(AC_NORMAL, 2)
ACCESS_MASK = (1 << 64) - 1
AIR_FRAME_BITS = base.AIR_FRAME_BITS
AIR_ACCESS_LEN = base.AIR_ACCESS_LEN
AIR_HEADER_LEN = base.AIR_HEADER_LEN
AIR_PAYLOAD_LEN = base.AIR_PAYLOAD_LEN
HEADER_OFFICIAL = base.HEADER_OFFICIAL
HEADER_OFFICIAL_BITS = base.bytes_to_bits(HEADER_OFFICIAL)

base.ENABLE_GRC_WATCHDOG = ENABLE_GRC_WATCHDOG
base.GRC_SCRIPT_PATH = GRC_SCRIPT_PATH
base.UDP_WATCHDOG_S = UDP_WATCHDOG_S
base.GRC_BOOT_WAIT_S = GRC_BOOT_WAIT_S
base.RPC_URL = RPC_URL

# 组帧/发布与 f3 共用
find_valid_frames = f3.find_valid_frames
drain_strict_frames = f3.drain_strict_frames
handle_valid_frame = f3.handle_valid_frame
f3.FRAME_DEDUP_TTL_S = FRAME_DEDUP_TTL_S


class AdaptiveController:
    """
    现场“自动曝光”控制器。

    每 2 秒读一次统计，决定要不要经 XMLRPC 换 RUNTIME_TUNES 里的档：
      explore — 还没稳定出帧，按表轮换 balanced→wide_fir→…
      locked  — 已经连续出帧，先别乱动；长时间又没帧再探索

    Access 容错（2/4）在本地改，不需要重启 GNU Radio。
    换 RF/FIR 档会清空比特缓冲（大改接收机后旧 0/1 不可信）。
    """

    def __init__(self, start_tune: str = "balanced") -> None:
        if start_tune not in RUNTIME_TUNES:
            start_tune = "balanced"
        self.tune_name = start_tune
        self.tune_index = (
            AUTO_TUNE_ORDER.index(start_tune)
            if start_tune in AUTO_TUNE_ORDER
            else 0
        )
        self.state = "explore"
        self.access_max = ACCESS_MAX_HAMMING_LOOSE
        self.good_windows = 0
        self.last_switch_ts = 0.0
        self.last_action = "init"
        self.switches = 0

    def _apply_tune(self, node: "InfoDecoderNode", name: str, now: float, reason: str) -> bool:
        if name not in RUNTIME_TUNES:
            return False
        if node.grc_rpc is None:
            self.last_action = f"skip:{reason}:no_rpc"
            return False
        tune = RUNTIME_TUNES[name]
        try:
            applied = node.grc_rpc.apply_runtime_tune(
                name,
                float(tune["rx_gain_db"]),
                int(tune["rf_bandwidth_hz"]),
                float(tune["fir_cutoff_hz"]),
                float(tune["fir_transition_hz"]),
            )
        except Exception as exc:
            self.last_action = f"fail:{reason}:{exc}"
            node.get_logger().warn(f"自适应切档失败 {name}: {exc}")
            return False
        self.tune_name = str(applied)
        if self.tune_name in AUTO_TUNE_ORDER:
            self.tune_index = AUTO_TUNE_ORDER.index(self.tune_name)
        self.last_switch_ts = now
        self.switches += 1
        self.last_action = f"{reason}->{self.tune_name}"
        node.get_logger().info(
            f"[f5-auto] {reason} → tune={self.tune_name} "
            f"gain={tune['rx_gain_db']} BW={tune['rf_bandwidth_hz']/1e3:.0f}k "
            f"FIR={tune['fir_cutoff_hz']/1e3:.0f}/{tune['fir_transition_hz']/1e3:.0f}k "
            f"Access≤{self.access_max}"
        )
        node._flush_rx_buffers = True
        return True

    def _next_explore_tune(self) -> str:
        self.tune_index = (self.tune_index + 1) % len(AUTO_TUNE_ORDER)
        return AUTO_TUNE_ORDER[self.tune_index]

    def _next_gain_step(self) -> str:
        """在同带宽/同 FIR 的档位里找下一个更高增益的档；到顶返回空串。"""
        cur = RUNTIME_TUNES.get(self.tune_name)
        if cur is None:
            return "weak_boost"
        cur_gain = float(cur["rx_gain_db"])
        candidates = [
            (float(t["rx_gain_db"]), name)
            for name, t in RUNTIME_TUNES.items()
            if name in AUTO_TUNE_ORDER
            and float(t["rx_gain_db"]) > cur_gain
            and int(t["rf_bandwidth_hz"]) == int(cur["rf_bandwidth_hz"])
        ]
        if not candidates:
            return ""
        return min(candidates)[1]

    def update(
        self,
        node: "InfoDecoderNode",
        stats: dict,
        *,
        now: float,
        silence_s: float,
        elapsed_s: float,
    ) -> None:
        """
        根据刚过去这一窗的统计做决策。

        判读口诀（小白版）：
          - 有有效帧 → 保持，争取进入 locked
          - 有 Access 无 Header → 滤波可能切太狠，放宽 FIR
          - 有 Header 无完整帧 → Payload 烂了，试抬增益或收窄邻道
          - 其它长时间无帧 → 按 AUTO_TUNE_ORDER 轮换（增益单调往上爬）

        这里刻意**不做**削顶推断。f5 没有 IQ 探头，唯一能看到的 udp_bytes 是
        GNU Radio 判决后的比特流量：即使天线上什么都没有，噪声也会被硬判成
        0/1 源源不断地发过来，所以“udp_bytes 大 + Access=0”是纯噪声的常态，
        而不是削顶的证据。旧版据此切到 desense(25dB) 会把弱信息波直接埋掉。
        需要判定削顶请用 f6（有 IQ 探头）。
        """
        frames = int(stats.get("strict_frames", 0)) + int(
            stats.get("window_frames", 0)
        )
        ac = int(stats.get("ac_hits", 0))
        header_ok = int(stats.get("header_ok", 0))
        header_fail = int(stats.get("header_fail", 0))
        crc16_fail = int(stats.get("crc16_fail", 0))

        # Access 容错本地可调，不依赖 RF RPC。
        if ac >= 8 and frames == 0 and crc16_fail >= 3:
            self.access_max = ACCESS_MAX_HAMMING_TIGHT
        elif ac == 0 and silence_s >= AUTO_EXPLORE_SILENCE_S:
            self.access_max = ACCESS_MAX_HAMMING_LOOSE
        elif frames > 0:
            self.access_max = ACCESS_MAX_HAMMING_LOOSE

        if not ENABLE_AUTO_TUNE or F5_PROFILE != "auto":
            self.last_action = "rf-auto-disabled"
            return

        if frames > 0:
            self.good_windows += 1
            if self.good_windows >= AUTO_LOCK_GOOD_WINDOWS:
                self.state = "locked"
            self.last_action = f"hold:{self.tune_name}:frames={frames}"
            return

        self.good_windows = 0

        if self.state == "locked":
            if silence_s < AUTO_RELOCK_SILENCE_S:
                self.last_action = f"locked-wait:{silence_s:.1f}s"
                return
            self.state = "explore"
            self.last_action = "relock->explore"

        if now - self.last_switch_ts < AUTO_COOLDOWN_S:
            self.last_action = f"cooldown:{AUTO_COOLDOWN_S - (now - self.last_switch_ts):.1f}s"
            return

        if silence_s < AUTO_EXPLORE_SILENCE_S:
            self.last_action = f"observe:{silence_s:.1f}s"
            return

        # 有 Access 无 Header：滤波可能过陡或边带被切。
        if ac >= 4 and header_ok == 0 and header_fail >= ac // 2:
            target = "wide_fir" if self.tune_name != "wide_fir" else "weak_boost"
            self._apply_tune(node, target, now, "header-starve")
            return

        # 有 Header 但帧组不起来：Payload 信噪比不够，沿增益阶梯往上爬一格。
        if header_ok >= 2 and frames == 0:
            target = self._next_gain_step()
            if target:
                self._apply_tune(node, target, now, "payload-starve")
                return

        # 通用轮换。
        nxt = self._next_explore_tune()
        self._apply_tune(
            node,
            nxt,
            now,
            f"explore:{elapsed_s:.1f}s/sil={silence_s:.1f}s",
        )


class InfoDecoderNode(base.Node):
    """保持 f3/f4 的 ROS 接口，节点名改为 info_decoder_f5。"""

    def __init__(self) -> None:
        super().__init__("info_decoder_f5")
        self.ally_camp = MY_CAMP
        self.grc_rpc: Optional[xmlrpc.client.ServerProxy] = None
        self._flush_rx_buffers = False
        start_tune = "balanced"
        if F5_PROFILE == "baseline":
            start_tune = "open"
        elif F5_PROFILE == "weak_fixed":
            start_tune = "weak_boost"
        self.auto = AdaptiveController(start_tune=start_tune)

        self.pub_pos = self.create_publisher(String, "radio/info/position", 10)
        self.pub_hp = self.create_publisher(String, "radio/info/hp", 10)
        self.pub_ammo = self.create_publisher(String, "radio/info/ammo", 10)
        self.pub_macro = self.create_publisher(String, "radio/info/macro", 10)
        self.pub_buff = self.create_publisher(String, "radio/info/buff", 10)
        self.create_subscription(Int8, "/team", self.team_callback, 10)

        self.get_logger().info(
            f"信息波 f5 启动 | 阵营={self.ally_camp} profile={F5_PROFILE} | "
            f"auto_tune={ENABLE_AUTO_TUNE} Access≤{ACCESS_MAX_HAMMING_LOOSE}bit | "
            "Header严格匹配 | 正式话题仅发布 CRC 有效帧"
        )

    def publish_json(self, publisher, data_dict: dict) -> None:
        msg = String()
        msg.data = base.json.dumps(data_dict, ensure_ascii=False)
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
        self.get_logger().info(f"阵营切换 → {new_camp}（听己方基座广播频点）")
        self.apply_camp_to_radio()
        self._flush_rx_buffers = True

    def apply_camp_to_radio(self) -> bool:
        if self.grc_rpc is None:
            return False
        freq = INFO_FREQ_MAP[self.ally_camp]
        try:
            self.grc_rpc.set_target_freq(freq)
        except Exception as exc:
            self.get_logger().error(f"切频失败: {exc}")
            return False
        try:
            self.grc_rpc.set_target_sens(INFO_SENSITIVITY)
        except Exception as exc:
            self.get_logger().warn(f"频点已切换，但 Sens RPC 失败: {exc}")
        # 切频后重新应用当前自适应档，避免只改频点丢掉 RF 设定。
        tune = RUNTIME_TUNES.get(self.auto.tune_name)
        if tune is not None:
            try:
                self.grc_rpc.apply_runtime_tune(
                    self.auto.tune_name,
                    float(tune["rx_gain_db"]),
                    int(tune["rf_bandwidth_hz"]),
                    float(tune["fir_cutoff_hz"]),
                    float(tune["fir_transition_hz"]),
                )
            except Exception as exc:
                self.get_logger().warn(f"切频后恢复 tune 失败: {exc}")
        self.get_logger().info(
            f"tx_radio5 已切至 {freq/1e6:.3f}MHz；tune={self.auto.tune_name}"
        )
        return True


def _fresh_stats(access_max: int) -> dict:
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "access_hamming": {i: 0 for i in range(access_max + 1)},
        "header_hamming": {i: 0 for i in range(HEADER_MAX_HAMMING + 1)},
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


def _find_access(
    bit_buffer: str, access_max: int
) -> Optional[tuple[int, bool, int]]:
    if len(bit_buffer) < 64:
        return None
    window = int(bit_buffer[:64], 2)
    last_start = len(bit_buffer) - 64
    for offset in range(last_start + 1):
        normal_distance = (window ^ AC_NORMAL_INT).bit_count()
        if normal_distance <= access_max:
            return offset, False, normal_distance
        inverted_distance = 64 - normal_distance
        if inverted_distance <= access_max:
            return offset, True, inverted_distance
        if offset < last_start:
            window = ((window << 1) & ACCESS_MASK) | (
                bit_buffer[offset + 64] == "1"
            )
    return None


def _header_hamming(header_bits: str) -> int:
    return base.hamming(header_bits, HEADER_OFFICIAL_BITS)


def extract_air_payloads(
    bit_buffer: str, stats: dict, access_max: int
) -> tuple[str, list[bytes]]:
    """
    拆空口 Payload；access_max 可由自适应控制器在 2～4 之间切换。
    Header 仍必须精确匹配（与 f4 相同铁律）。
    """
    payloads: list[bytes] = []
    while len(bit_buffer) >= AIR_FRAME_BITS:
        found = _find_access(bit_buffer, access_max)
        if found is None:
            bit_buffer = bit_buffer[-63:]
            break

        start, inverted, access_distance = found
        if len(bit_buffer) < start + AIR_FRAME_BITS:
            bit_buffer = bit_buffer[start:]
            break

        raw_bits = bit_buffer[start:start + AIR_FRAME_BITS]
        frame_bits = base.invert_bits(raw_bits) if inverted else raw_bits
        header_start = AIR_ACCESS_LEN * 8
        header_end = header_start + AIR_HEADER_LEN * 8
        header_distance = _header_hamming(frame_bits[header_start:header_end])

        stats["ac_hits"] += 1
        stats["access_hamming"].setdefault(access_distance, 0)
        stats["access_hamming"][access_distance] += 1
        if inverted:
            stats["ac_hits_inverted"] += 1

        if header_distance <= HEADER_MAX_HAMMING:
            frame_bytes = base.bits_to_bytes(frame_bits)
            payloads.append(
                frame_bytes[
                    AIR_ACCESS_LEN + AIR_HEADER_LEN:
                    AIR_ACCESS_LEN + AIR_HEADER_LEN + AIR_PAYLOAD_LEN
                ]
            )
            stats["header_ok"] += 1
            stats["header_hamming"].setdefault(header_distance, 0)
            stats["header_hamming"][header_distance] += 1
            stats["last_polarity"] = "inverted" if inverted else "normal"
            bit_buffer = bit_buffer[start + AIR_FRAME_BITS:]
        else:
            stats["header_fail"] += 1
            bit_buffer = bit_buffer[start + 1:]
    return bit_buffer, payloads


def _build_test_round(seq_start: int) -> bytes:
    pos = struct.pack("<12H", 100, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    hp = struct.pack("<6H", 2000, 500, 800, 800, 0, 600)
    ammo = struct.pack("<5H", 150, 450, 400, 500, 800)
    macro = struct.pack("<HHI", 350, 1200, 256)
    buff = bytearray(41)
    buff[35] = 4
    return b"".join(
        (
            base.build_referee_frame(0x0A01, pos, seq_start + 0),
            base.build_referee_frame(0x0A02, hp, seq_start + 1),
            base.build_referee_frame(0x0A03, ammo, seq_start + 2),
            base.build_referee_frame(0x0A04, macro, seq_start + 3),
            base.build_referee_frame(0x0A05, bytes(buff), seq_start + 4),
        )
    )


def _wrap_continuous_air_bits(serial_stream: bytes) -> str:
    if len(serial_stream) % AIR_PAYLOAD_LEN:
        raise ValueError("连续流测试必须在整体末尾恰好落在 15B 边界")
    packets = []
    for offset in range(0, len(serial_stream), AIR_PAYLOAD_LEN):
        packets.append(
            bytes.fromhex(base.ACCESS_CODE_HEX)
            + HEADER_OFFICIAL
            + serial_stream[offset:offset + AIR_PAYLOAD_LEN]
        )
    return "".join(base.bytes_to_bits(packet) for packet in packets)


def _flip_bits(bits: str, positions: tuple[int, ...]) -> str:
    changed = list(bits)
    for position in positions:
        changed[position] = "1" if changed[position] == "0" else "0"
    return "".join(changed)


def run_self_test(node: InfoDecoderNode) -> bool:
    """
    纯内存自检：连续切片、Access 容错、坏 Header、CRC，以及 Access 自适应收紧。
    不调用业务 publisher，避免假数据进正式话题。
    """
    node.get_logger().info("==== f5 纯内存自检开始 ====")
    rounds = b"".join(_build_test_round(index * 5) for index in range(3))
    if len(rounds) != 420:
        node.get_logger().error(f"连续三轮长度错误: {len(rounds)}B，应为420B")
        return False

    clean_bits = _wrap_continuous_air_bits(rounds)
    access_damaged = _flip_bits(clean_bits, (3, 20, 41, 55))

    stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    remain, payloads = extract_air_payloads(
        access_damaged, stats, ACCESS_MAX_HAMMING_LOOSE
    )
    extracted = b"".join(payloads)
    access_ok = len(payloads) == 28 and not remain and extracted == rounds

    second_header = AIR_FRAME_BITS + 64
    header_damaged = _flip_bits(clean_bits, (second_header,))
    header_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    header_remain, header_payloads = extract_air_payloads(
        header_damaged, header_stats, ACCESS_MAX_HAMMING_LOOSE
    )
    expected_without_second_payload = rounds[:15] + rounds[30:]
    strict_header_ok = (
        len(header_payloads) == 27
        and not header_remain
        and b"".join(header_payloads) == expected_without_second_payload
        and header_stats["header_fail"] == 1
    )

    # 自适应：假同步迹象应收紧 Access。
    fake_stats = {
        "ac_hits": 12,
        "strict_frames": 0,
        "window_frames": 0,
        "crc16_fail": 5,
        "header_ok": 0,
        "header_fail": 0,
        "udp_bytes": 1000,
    }
    ctrl = AdaptiveController("balanced")
    ctrl.access_max = ACCESS_MAX_HAMMING_LOOSE
    ctrl.update(
        node,
        fake_stats,
        now=time.time(),
        silence_s=1.0,
        elapsed_s=2.0,
    )
    access_adapt_ok = ctrl.access_max == ACCESS_MAX_HAMMING_TIGHT

    strict_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    strict_frames = drain_strict_frames(bytearray(extracted), strict_stats)
    window_frames = find_valid_frames(extracted)
    cmd_ids = [struct.unpack_from("<H", frame, 5)[0] for frame in strict_frames]
    frame_ok = (
        len(strict_frames) == 15
        and len(window_frames) == 15
        and cmd_ids.count(0x0A05) == 3
    )
    ok = access_ok and strict_header_ok and frame_ok and access_adapt_ok

    node.get_logger().info(
        f"自检 连续切片={'通过' if access_ok else '失败'} "
        f"严格Header={'通过' if strict_header_ok else '失败'} "
        f"15帧/0x0A05={'通过' if frame_ok else '失败'} "
        f"Access自适应={'通过' if access_adapt_ok else '失败'}"
    )
    node.get_logger().info(
        "==== f5 纯内存自检结束：%s ====" % ("全部通过 OK" if ok else "存在失败 FAIL")
    )
    return ok


def connect_grc(node: InfoDecoderNode) -> None:
    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
            try:
                backend = node.grc_rpc.get_clock_backend()
                node.get_logger().info(f"tx_radio5 时钟后端={backend}")
            except Exception:
                pass
        else:
            node.get_logger().error("XMLRPC 调用失败，请先启动 tx_radio5.py")
    except Exception as exc:
        node.get_logger().error(f"无法连接 XMLRPC({RPC_URL}): {exc}")


def main() -> None:
    """主循环：收比特 → 拆包组帧 → 每 2s 让 AdaptiveController 决定是否换档。"""
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("f5 离线自检未通过，请勿直接用于赛场！")

    # tx_radio5.py 和本文件读的是同一个 tx_radio5_tunes.BOOT_PROFILE，
    # 不需要再靠环境变量把 profile 传给子进程。
    if ENABLE_GRC_WATCHDOG:
        base.restart_grc(node)
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
        node.destroy_node()
        rclpy.shutdown()
        return

    bit_buffer = ""
    serial_buffer = bytearray()
    packet_window: deque[bytes] = deque(maxlen=PACKET_WINDOW_PAYLOADS)
    dedupe: dict[tuple[int, int, int], float] = {}
    packet_counter = {
        key: 0 for key in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")
    }
    stats = _fresh_stats(node.auto.access_max)
    last_stat = time.time()
    last_udp = time.time()
    last_valid_frame = time.time()

    node.get_logger().info(
        f"监听 {UDP_IP}:{UDP_PORT} | GRC={GRC_SCRIPT_PATH} | "
        f"profile={F5_PROFILE} tune={node.auto.tune_name}"
    )

    try:
        while rclpy.ok():
            now = time.time()
            if node._flush_rx_buffers:
                node._flush_rx_buffers = False
                bit_buffer = ""
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()

            if ENABLE_GRC_WATCHDOG and now - last_udp > UDP_WATCHDOG_S:
                node.get_logger().warn("UDP 断流，重启 tx_radio5…")
                base.restart_grc(node)
                last_udp = time.time()
                bit_buffer = ""
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()
                # 重启后把当前自适应档重新打上去。
                tune = RUNTIME_TUNES.get(node.auto.tune_name)
                if node.grc_rpc is not None and tune is not None:
                    try:
                        node.grc_rpc.apply_runtime_tune(
                            node.auto.tune_name,
                            float(tune["rx_gain_db"]),
                            int(tune["rf_bandwidth_hz"]),
                            float(tune["fir_cutoff_hz"]),
                            float(tune["fir_transition_hz"]),
                        )
                    except Exception as exc:
                        node.get_logger().warn(f"重启后恢复 tune 失败: {exc}")

            if now - last_stat >= STAT_INTERVAL_S:
                elapsed = max(0.001, now - last_stat)
                silence = now - last_valid_frame
                rates = " ".join(
                    f"{key}={count/elapsed:.1f}Hz"
                    for key, count in packet_counter.items()
                )
                access_cap = node.auto.access_max
                access_hist = "/".join(
                    str(stats["access_hamming"].get(i, 0))
                    for i in range(access_cap + 1)
                )
                node.auto.update(
                    node,
                    stats,
                    now=now,
                    silence_s=silence,
                    elapsed_s=elapsed,
                )
                node.get_logger().info(
                    f"[f5 {F5_PROFILE}/{node.auto.tune_name} "
                    f"{node.auto.state} {elapsed:.1f}s] {rates} | "
                    f"AC0..{access_cap}={access_hist} "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16={stats['crc8_fail']}/{stats['crc16_fail']} "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"无有效帧={silence:.1f}s UDP={stats['udp_bytes']} | "
                    f"auto={node.auto.last_action}"
                )
                packet_counter = {key: 0 for key in packet_counter}
                polarity = stats["last_polarity"]
                stats = _fresh_stats(node.auto.access_max)
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
                    incoming = "".join("1" if byte else "0" for byte in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]

            bit_buffer, new_payloads = extract_air_payloads(
                bit_buffer, stats, node.auto.access_max
            )
            if new_payloads:
                packet_window.extend(new_payloads)
                serial_buffer.extend(b"".join(new_payloads))
                if len(serial_buffer) > SERIAL_BUFFER_MAX:
                    del serial_buffer[:-SERIAL_BUFFER_MAX]

            published = False
            for frame in drain_strict_frames(serial_buffer, stats):
                published = (
                    handle_valid_frame(
                        frame, "strict", node, packet_counter, stats, dedupe, now
                    )
                    or published
                )

            if ENABLE_PACKET_WINDOW_RECOVERY and new_payloads:
                for frame in find_valid_frames(b"".join(packet_window)):
                    published = (
                        handle_valid_frame(
                            frame, "window", node, packet_counter, stats, dedupe, now
                        )
                        or published
                    )
            if published:
                last_valid_frame = now

            rclpy.spin_once(node, timeout_sec=0.0)
            if not got_udp:
                time.sleep(IDLE_SLEEP_S)
    except KeyboardInterrupt:
        node.get_logger().info("中断退出")
    finally:
        if base._grc_process is not None:
            base._kill_grc_tree(base._grc_process)
        if file_recorder is not None:
            file_recorder.flush()
            file_recorder.close()
        sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
