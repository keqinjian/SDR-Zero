#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboMaster 2026 信息波近最优解码器 f6。

在 f5 之上增加：
  1. IQ/FM 探头驱动的连续 AGC（小步调增益不清缓冲）；
  2. soft float UDP 上的 Access 软相关同步；
  3. Header 仍严格硬匹配；CRC 有效才发布；
  4. 离散 RF/FIR 档仅作兜底大步探索。
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
from tx_radio6_tunes import (
    AGC_GAIN_MAX_DB,
    AGC_GAIN_MIN_DB,
    AGC_STEP_DB,
    AUTO_TUNE_ORDER,
    IQ_CLIP_MAX_ABS,
    IQ_CLIP_RMS,
    IQ_WEAK_MAX_ABS,
    RUNTIME_TUNES,
)


MY_CAMP = "RED"
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
SOFT_UDP_PORT = int(os.environ.get("INFO_F6_SOFT_UDP_PORT", "14347"))
RPC_URL = "http://127.0.0.1:8081"

F6_PROFILE = os.environ.get("INFO_F6_PROFILE", "auto").strip().lower()
ENABLE_AUTO = os.environ.get("INFO_F6_AUTO", "1") not in (
    "0", "false", "False", "no", "NO",
)
ENABLE_SOFT_SYNC = os.environ.get("INFO_F6_SOFT_SYNC", "1") not in (
    "0", "false", "False", "no", "NO",
)
ACCESS_MAX_HAMMING_LOOSE = int(os.environ.get("INFO_F6_ACCESS_HAMMING", "4"))
ACCESS_MAX_HAMMING_TIGHT = 2
HEADER_MAX_HAMMING = 0
SOFT_CORR_MIN = float(os.environ.get("INFO_F6_SOFT_CORR_MIN", "40"))

ENABLE_PACKET_WINDOW_RECOVERY = True
PACKET_WINDOW_PAYLOADS = 8
FRAME_DEDUP_TTL_S = 2.0
ENABLE_GRC_WATCHDOG = os.environ.get("RM_ENABLE_GRC_WATCHDOG", "1") not in (
    "0", "false", "False", "no", "NO",
)
GRC_SCRIPT_PATH = os.environ.get(
    "INFO_GRC_SCRIPT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "tx_radio6.py"),
)
UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.5
RECORD_STREAM = os.environ.get("INFO_F6_RECORD", "0") in (
    "1", "true", "True", "yes", "YES",
)
RECORD_PREFIX = "info_f6_record"
SELF_TEST_ON_START = True
STAT_INTERVAL_S = 2.0
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 240_000
SOFT_BUFFER_MAX = 240_000
SERIAL_BUFFER_MAX = 16_384

AUTO_COOLDOWN_S = float(os.environ.get("INFO_F6_AUTO_COOLDOWN_S", "6"))
AUTO_EXPLORE_SILENCE_S = float(os.environ.get("INFO_F6_AUTO_SILENCE_S", "4"))
AUTO_LOCK_GOOD_WINDOWS = 3
AUTO_RELOCK_SILENCE_S = 10.0
AGC_COOLDOWN_S = float(os.environ.get("INFO_F6_AGC_COOLDOWN_S", "2"))

INFO_FREQ_MAP = base.INFO_FREQ_MAP
INFO_SENSITIVITY = base.INFO_SENSITIVITY
AC_NORMAL = base.AC_NORMAL
AC_NORMAL_INT = int(AC_NORMAL, 2)
ACCESS_MASK = (1 << 64) - 1
ACCESS_PM = tuple(1.0 if bit == "1" else -1.0 for bit in AC_NORMAL)
AIR_FRAME_BITS = base.AIR_FRAME_BITS
AIR_ACCESS_LEN = base.AIR_ACCESS_LEN
AIR_HEADER_LEN = base.AIR_HEADER_LEN
AIR_PAYLOAD_LEN = base.AIR_PAYLOAD_LEN
AIR_FRAME_SYMBOLS = AIR_FRAME_BITS
HEADER_OFFICIAL = base.HEADER_OFFICIAL
HEADER_OFFICIAL_BITS = base.bytes_to_bits(HEADER_OFFICIAL)

base.ENABLE_GRC_WATCHDOG = ENABLE_GRC_WATCHDOG
base.GRC_SCRIPT_PATH = GRC_SCRIPT_PATH
base.UDP_WATCHDOG_S = UDP_WATCHDOG_S
base.GRC_BOOT_WAIT_S = GRC_BOOT_WAIT_S
base.RPC_URL = RPC_URL

find_valid_frames = f3.find_valid_frames
drain_strict_frames = f3.drain_strict_frames
handle_valid_frame = f3.handle_valid_frame
f3.FRAME_DEDUP_TTL_S = FRAME_DEDUP_TTL_S


def _clip_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    return float(raw)


IQ_CLIP_MAX_ABS_EFF = _clip_env("INFO_F6_IQ_CLIP_MAX_ABS", IQ_CLIP_MAX_ABS)
IQ_WEAK_MAX_ABS_EFF = _clip_env("INFO_F6_IQ_WEAK_MAX_ABS", IQ_WEAK_MAX_ABS)
IQ_CLIP_RMS_EFF = _clip_env("INFO_F6_IQ_CLIP_RMS", IQ_CLIP_RMS)


class AdaptiveController:
    """探头驱动连续 AGC + 离散档兜底；小步 AGC 不 flush。"""

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
        self.last_agc_ts = 0.0
        self.last_action = "init"
        self.switches = 0
        self.agc_gain = float(RUNTIME_TUNES[start_tune]["rx_gain_db"])
        self.last_iq = {"count": 0, "max_abs": 0.0, "rms": 0.0}
        self.last_fm = {"count": 0, "estimated_cfo_hz": 0.0, "mean": 0.0}

    def _rf_auto_enabled(self) -> bool:
        return ENABLE_AUTO and F6_PROFILE == "auto"

    def _apply_tune(
        self,
        node: "InfoDecoderNode",
        name: str,
        now: float,
        reason: str,
        *,
        flush: bool,
    ) -> bool:
        if name not in RUNTIME_TUNES or node.grc_rpc is None:
            self.last_action = f"skip:{reason}"
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
            node.get_logger().warn(f"f6 切档失败 {name}: {exc}")
            return False
        self.tune_name = str(applied)
        self.agc_gain = float(tune["rx_gain_db"])
        if self.tune_name in AUTO_TUNE_ORDER:
            self.tune_index = AUTO_TUNE_ORDER.index(self.tune_name)
        self.last_switch_ts = now
        self.switches += 1
        self.last_action = f"{reason}->{self.tune_name}"
        node.get_logger().info(
            f"[f6-auto] {reason} → tune={self.tune_name} "
            f"gain={self.agc_gain:.1f} Access≤{self.access_max} flush={flush}"
        )
        if flush:
            node._flush_rx_buffers = True
        return True

    def _apply_agc(
        self, node: "InfoDecoderNode", gain: float, now: float, reason: str
    ) -> bool:
        if node.grc_rpc is None:
            return False
        gain = max(AGC_GAIN_MIN_DB, min(AGC_GAIN_MAX_DB, float(gain)))
        if abs(gain - self.agc_gain) < 0.25:
            return False
        try:
            applied = float(node.grc_rpc.apply_agc_gain(gain))
        except Exception as exc:
            self.last_action = f"agc-fail:{exc}"
            return False
        self.agc_gain = applied
        self.last_agc_ts = now
        self.last_action = f"agc:{reason}->{applied:.1f}dB"
        node.get_logger().info(f"[f6-agc] {reason} → gain={applied:.1f}dB")
        return True

    def _poll_probes(self, node: "InfoDecoderNode") -> None:
        if node.grc_rpc is None:
            return
        try:
            self.last_iq = dict(node.grc_rpc.get_iq_stats())
        except Exception:
            pass
        try:
            self.last_fm = dict(node.grc_rpc.get_fm_stats())
        except Exception:
            pass

    def _next_explore_tune(self) -> str:
        self.tune_index = (self.tune_index + 1) % len(AUTO_TUNE_ORDER)
        return AUTO_TUNE_ORDER[self.tune_index]

    def update(
        self,
        node: "InfoDecoderNode",
        stats: dict,
        *,
        now: float,
        silence_s: float,
        elapsed_s: float,
    ) -> None:
        frames = int(stats.get("strict_frames", 0)) + int(
            stats.get("window_frames", 0)
        )
        ac = int(stats.get("ac_hits", 0))
        header_ok = int(stats.get("header_ok", 0))
        header_fail = int(stats.get("header_fail", 0))
        crc16_fail = int(stats.get("crc16_fail", 0))

        if ac >= 8 and frames == 0 and crc16_fail >= 3:
            self.access_max = ACCESS_MAX_HAMMING_TIGHT
        elif ac == 0 and silence_s >= AUTO_EXPLORE_SILENCE_S:
            self.access_max = ACCESS_MAX_HAMMING_LOOSE
        elif frames > 0:
            self.access_max = ACCESS_MAX_HAMMING_LOOSE

        self._poll_probes(node)
        iq = self.last_iq
        max_abs = float(iq.get("max_abs", 0.0) or 0.0)
        rms = float(iq.get("rms", 0.0) or 0.0)
        iq_count = int(iq.get("count", 0) or 0)

        if not self._rf_auto_enabled():
            self.last_action = "auto-disabled"
            return

        # 连续 AGC：削顶优先降增益（不清缓冲）
        if (
            iq_count > 0
            and now - self.last_agc_ts >= AGC_COOLDOWN_S
            and (max_abs >= IQ_CLIP_MAX_ABS_EFF or rms >= IQ_CLIP_RMS_EFF)
        ):
            self._apply_agc(
                node, self.agc_gain - AGC_STEP_DB, now, "clip"
            )
            return

        if frames > 0:
            self.good_windows += 1
            if self.good_windows >= AUTO_LOCK_GOOD_WINDOWS:
                self.state = "locked"
            # locked/有帧时仅微调：过弱则小升增益
            if (
                iq_count > 0
                and max_abs < IQ_WEAK_MAX_ABS_EFF
                and now - self.last_agc_ts >= AGC_COOLDOWN_S
            ):
                self._apply_agc(
                    node, self.agc_gain + AGC_STEP_DB * 0.5, now, "weak-lock"
                )
                return
            self.last_action = (
                f"hold:{self.tune_name}:g={self.agc_gain:.1f}:frames={frames}"
            )
            return

        self.good_windows = 0

        if self.state == "locked":
            if silence_s < AUTO_RELOCK_SILENCE_S:
                if (
                    iq_count > 0
                    and max_abs < IQ_WEAK_MAX_ABS_EFF
                    and now - self.last_agc_ts >= AGC_COOLDOWN_S
                ):
                    self._apply_agc(
                        node, self.agc_gain + AGC_STEP_DB, now, "weak-silence"
                    )
                    return
                self.last_action = f"locked-wait:{silence_s:.1f}s"
                return
            self.state = "explore"

        if now - self.last_switch_ts < AUTO_COOLDOWN_S:
            # 冷却期内仍允许 AGC
            if (
                iq_count > 0
                and max_abs < IQ_WEAK_MAX_ABS_EFF
                and ac == 0
                and now - self.last_agc_ts >= AGC_COOLDOWN_S
            ):
                self._apply_agc(
                    node, self.agc_gain + AGC_STEP_DB, now, "weak-explore"
                )
                return
            self.last_action = "cooldown"
            return

        if silence_s < AUTO_EXPLORE_SILENCE_S:
            self.last_action = f"observe:{silence_s:.1f}s"
            return

        # 探头确认削顶 → desense 档（FIR/BW 也收）
        if iq_count > 0 and (
            max_abs >= IQ_CLIP_MAX_ABS_EFF or rms >= IQ_CLIP_RMS_EFF
        ):
            if self.tune_name != "desense":
                self._apply_tune(node, "desense", now, "probe-clip", flush=True)
                return
            self._apply_agc(node, self.agc_gain - AGC_STEP_DB, now, "probe-clip")
            return

        if ac >= 4 and header_ok == 0 and header_fail >= max(1, ac // 2):
            target = "wide_fir" if self.tune_name != "wide_fir" else "weak_boost"
            self._apply_tune(node, target, now, "header-starve", flush=True)
            return

        if header_ok >= 2 and frames == 0:
            target = "weak_boost" if self.tune_name != "weak_boost" else "narrow"
            self._apply_tune(node, target, now, "payload-starve", flush=True)
            return

        if iq_count > 0 and max_abs < IQ_WEAK_MAX_ABS_EFF and ac == 0:
            self._apply_agc(node, self.agc_gain + AGC_STEP_DB, now, "weak-noac")
            return

        nxt = self._next_explore_tune()
        self._apply_tune(
            node,
            nxt,
            now,
            f"explore:sil={silence_s:.1f}s",
            flush=True,
        )


class InfoDecoderNode(base.Node):
    def __init__(self) -> None:
        super().__init__("info_decoder_f6")
        self.ally_camp = MY_CAMP
        self.grc_rpc: Optional[xmlrpc.client.ServerProxy] = None
        self._flush_rx_buffers = False
        start_tune = "balanced"
        if F6_PROFILE == "baseline":
            start_tune = "open"
        self.auto = AdaptiveController(start_tune=start_tune)

        self.pub_pos = self.create_publisher(String, "radio/info/position", 10)
        self.pub_hp = self.create_publisher(String, "radio/info/hp", 10)
        self.pub_ammo = self.create_publisher(String, "radio/info/ammo", 10)
        self.pub_macro = self.create_publisher(String, "radio/info/macro", 10)
        self.pub_buff = self.create_publisher(String, "radio/info/buff", 10)
        self.create_subscription(Int8, "/team", self.team_callback, 10)

        self.get_logger().info(
            f"信息波 f6 启动 | 阵营={self.ally_camp} profile={F6_PROFILE} | "
            f"auto={ENABLE_AUTO} soft={ENABLE_SOFT_SYNC} | "
            f"Access≤{ACCESS_MAX_HAMMING_LOOSE} Header严格 | CRC门闩"
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
        self.get_logger().info(f"阵营切换 → {new_camp}")
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
            self.get_logger().warn(f"Sens RPC 失败: {exc}")
        tune = RUNTIME_TUNES.get(self.auto.tune_name)
        if tune is not None:
            try:
                self.grc_rpc.apply_runtime_tune(
                    self.auto.tune_name,
                    float(self.auto.agc_gain),
                    int(tune["rf_bandwidth_hz"]),
                    float(tune["fir_cutoff_hz"]),
                    float(tune["fir_transition_hz"]),
                )
            except Exception as exc:
                self.get_logger().warn(f"切频后恢复 tune 失败: {exc}")
        self.get_logger().info(
            f"tx_radio6 → {freq/1e6:.3f}MHz tune={self.auto.tune_name} "
            f"gain={self.auto.agc_gain:.1f}"
        )
        return True


def _fresh_stats(access_max: int) -> dict:
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "ac_soft_hits": 0,
        "access_hamming": {i: 0 for i in range(access_max + 1)},
        "header_hamming": {0: 0},
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
        "soft_udp_bytes": 0,
        "last_soft_corr": 0.0,
    }


def _find_access_hard(
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


def _soft_corr_at(soft: list[float], offset: int) -> tuple[float, float]:
    normal = 0.0
    for i, pm in enumerate(ACCESS_PM):
        normal += soft[offset + i] * pm
    return normal, -normal


def find_access_soft(
    soft: list[float],
    *,
    corr_min: float,
    access_max: int,
) -> Optional[tuple[int, bool, float, int]]:
    """
    软相关找 Access（从左到右首个过阈）。返回 (offset, inverted, corr, hard_hamming)。
    """
    if len(soft) < 64:
        return None
    last = len(soft) - 64
    # 理想 ±1 时 corr≈64-2d；允许 d<=access_max
    thr = max(corr_min, float(64 - 2 * access_max) * 0.85)
    for offset in range(last + 1):
        normal, inverted_corr = _soft_corr_at(soft, offset)
        if normal >= inverted_corr:
            score, inv = normal, False
        else:
            score, inv = inverted_corr, True
        if score < thr:
            continue
        hard_bits = "".join(
            "1" if soft[offset + i] >= 0 else "0" for i in range(64)
        )
        if inv:
            hard_bits = base.invert_bits(hard_bits)
        dist = (int(hard_bits, 2) ^ AC_NORMAL_INT).bit_count()
        if dist > access_max and score < float(64 - 2 * access_max):
            continue
        return offset, inv, score, dist
    return None


def soft_to_hard_bits(soft: list[float], inverted: bool) -> str:
    bits = "".join("1" if sample >= 0 else "0" for sample in soft)
    return base.invert_bits(bits) if inverted else bits


def extract_air_payloads_soft(
    soft: list[float], stats: dict, access_max: int
) -> tuple[list[float], list[bytes]]:
    payloads: list[bytes] = []
    while len(soft) >= AIR_FRAME_SYMBOLS:
        found = find_access_soft(
            soft, corr_min=SOFT_CORR_MIN, access_max=access_max
        )
        if found is None:
            soft = soft[-(AIR_FRAME_SYMBOLS - 1) :]
            break
        start, inverted, corr, dist = found
        if len(soft) < start + AIR_FRAME_SYMBOLS:
            soft = soft[start:]
            break
        frame_soft = soft[start:start + AIR_FRAME_SYMBOLS]
        frame_bits = soft_to_hard_bits(frame_soft, inverted)
        header_start = AIR_ACCESS_LEN * 8
        header_end = header_start + AIR_HEADER_LEN * 8
        header_distance = base.hamming(
            frame_bits[header_start:header_end], HEADER_OFFICIAL_BITS
        )

        stats["ac_hits"] += 1
        stats["ac_soft_hits"] += 1
        stats["last_soft_corr"] = float(corr)
        stats["access_hamming"].setdefault(dist, 0)
        stats["access_hamming"][dist] += 1
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
            stats["header_hamming"][0] = stats["header_hamming"].get(0, 0) + 1
            stats["last_polarity"] = "inverted" if inverted else "normal"
            soft = soft[start + AIR_FRAME_SYMBOLS:]
        else:
            stats["header_fail"] += 1
            soft = soft[start + 1:]
    return soft, payloads


def extract_air_payloads_hard(
    bit_buffer: str, stats: dict, access_max: int
) -> tuple[str, list[bytes]]:
    payloads: list[bytes] = []
    while len(bit_buffer) >= AIR_FRAME_BITS:
        found = _find_access_hard(bit_buffer, access_max)
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
        header_distance = base.hamming(
            frame_bits[header_start:header_end], HEADER_OFFICIAL_BITS
        )
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


def _bits_to_soft(bits: str, amplitude: float = 1.0) -> list[float]:
    return [amplitude if bit == "1" else -amplitude for bit in bits]


def _flip_bits(bits: str, positions: tuple[int, ...]) -> str:
    changed = list(bits)
    for position in positions:
        changed[position] = "1" if changed[position] == "0" else "0"
    return "".join(changed)


def run_self_test(node: InfoDecoderNode) -> bool:
    node.get_logger().info("==== f6 纯内存自检开始 ====")
    rounds = b"".join(_build_test_round(index * 5) for index in range(3))
    if len(rounds) != 420:
        node.get_logger().error(f"连续三轮长度错误: {len(rounds)}B")
        return False

    clean_bits = _wrap_continuous_air_bits(rounds)
    access_damaged = _flip_bits(clean_bits, (3, 20, 41, 55))

    hard_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    remain, payloads = extract_air_payloads_hard(
        access_damaged, hard_stats, ACCESS_MAX_HAMMING_LOOSE
    )
    extracted = b"".join(payloads)
    hard_ok = len(payloads) == 28 and not remain and extracted == rounds

    soft = _bits_to_soft(access_damaged)
    # 对 Access 前若干 soft 加噪声，验证软相关仍可锁定
    for i in (3, 20, 41, 55):
        soft[i] *= 0.2
    soft_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    soft_remain, soft_payloads = extract_air_payloads_soft(
        soft, soft_stats, ACCESS_MAX_HAMMING_LOOSE
    )
    soft_ok = (
        len(soft_payloads) == 28
        and not soft_remain
        and b"".join(soft_payloads) == rounds
        and soft_stats["ac_soft_hits"] >= 28
    )

    second_header = AIR_FRAME_BITS + 64
    header_damaged = _flip_bits(clean_bits, (second_header,))
    header_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    header_remain, header_payloads = extract_air_payloads_hard(
        header_damaged, header_stats, ACCESS_MAX_HAMMING_LOOSE
    )
    expected = rounds[:15] + rounds[30:]
    header_ok = (
        len(header_payloads) == 27
        and not header_remain
        and b"".join(header_payloads) == expected
        and header_stats["header_fail"] == 1
    )

    ctrl = AdaptiveController("balanced")
    ctrl.access_max = ACCESS_MAX_HAMMING_LOOSE
    ctrl.update(
        node,
        {
            "ac_hits": 12,
            "strict_frames": 0,
            "window_frames": 0,
            "crc16_fail": 5,
            "header_ok": 0,
            "header_fail": 0,
            "udp_bytes": 0,
        },
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
    ok = hard_ok and soft_ok and header_ok and frame_ok and access_adapt_ok
    node.get_logger().info(
        f"自检 硬切片={'通过' if hard_ok else '失败'} "
        f"软相关={'通过' if soft_ok else '失败'} "
        f"严格Header={'通过' if header_ok else '失败'} "
        f"15帧={'通过' if frame_ok else '失败'} "
        f"Access自适应={'通过' if access_adapt_ok else '失败'}"
    )
    node.get_logger().info(
        "==== f6 纯内存自检结束：%s ====" % ("全部通过 OK" if ok else "存在失败 FAIL")
    )
    return ok


def connect_grc(node: InfoDecoderNode) -> None:
    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
            try:
                backend = node.grc_rpc.get_clock_backend()
                node.get_logger().info(f"时钟后端={backend}")
            except Exception:
                pass
        else:
            node.get_logger().error("XMLRPC 调用失败，请先启动 tx_radio6.py")
    except Exception as exc:
        node.get_logger().error(f"无法连接 XMLRPC({RPC_URL}): {exc}")


def _recv_soft_floats(sock: socket.socket, stats: dict) -> list[float]:
    values: list[float] = []
    try:
        while True:
            data, _ = sock.recvfrom(65535)
            stats["soft_udp_bytes"] += len(data)
            # 对齐到 4 字节
            usable = len(data) - (len(data) % 4)
            if usable <= 0:
                continue
            count = usable // 4
            values.extend(struct.unpack(f"<{count}f", data[:usable]))
    except BlockingIOError:
        pass
    return values


def main() -> None:
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("f6 离线自检未通过，请勿直接用于赛场！")

    os.environ["INFO_F6_PROFILE"] = F6_PROFILE
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

    hard_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hard_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    hard_sock.setblocking(False)
    soft_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soft_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    soft_sock.setblocking(False)
    try:
        hard_sock.bind((UDP_IP, UDP_PORT))
    except OSError as exc:
        node.get_logger().error(f"硬比特 UDP 绑定失败: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return
    soft_bound = False
    if ENABLE_SOFT_SYNC:
        try:
            soft_sock.bind((UDP_IP, SOFT_UDP_PORT))
            soft_bound = True
            node.get_logger().info(f"软符号 UDP {UDP_IP}:{SOFT_UDP_PORT}")
        except OSError as exc:
            node.get_logger().warn(f"软符号 UDP 绑定失败，回退硬路径: {exc}")

    bit_buffer = ""
    soft_buffer: list[float] = []
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
        f"监听 hard={UDP_PORT} soft={SOFT_UDP_PORT if soft_bound else 'off'} | "
        f"GRC={GRC_SCRIPT_PATH} | profile={F6_PROFILE}"
    )

    try:
        while rclpy.ok():
            now = time.time()
            if node._flush_rx_buffers:
                node._flush_rx_buffers = False
                bit_buffer = ""
                soft_buffer = []
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()

            if ENABLE_GRC_WATCHDOG and now - last_udp > UDP_WATCHDOG_S:
                node.get_logger().warn("UDP 断流，重启 tx_radio6…")
                os.environ["INFO_F6_PROFILE"] = F6_PROFILE
                base.restart_grc(node)
                last_udp = time.time()
                bit_buffer = ""
                soft_buffer = []
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()
                tune = RUNTIME_TUNES.get(node.auto.tune_name)
                if node.grc_rpc is not None and tune is not None:
                    try:
                        node.grc_rpc.apply_runtime_tune(
                            node.auto.tune_name,
                            float(node.auto.agc_gain),
                            int(tune["rf_bandwidth_hz"]),
                            float(tune["fir_cutoff_hz"]),
                            float(tune["fir_transition_hz"]),
                        )
                    except Exception as exc:
                        node.get_logger().warn(f"重启后恢复失败: {exc}")

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
                iq = node.auto.last_iq
                fm = node.auto.last_fm
                node.get_logger().info(
                    f"[f6 {F6_PROFILE}/{node.auto.tune_name} "
                    f"{node.auto.state} g={node.auto.agc_gain:.1f} "
                    f"{elapsed:.1f}s] {rates} | "
                    f"AC0..{access_cap}={access_hist} softAC={stats['ac_soft_hits']} "
                    f"corr={stats['last_soft_corr']:.1f} "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16={stats['crc8_fail']}/{stats['crc16_fail']} "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"无帧={silence:.1f}s | "
                    f"IQmax={float(iq.get('max_abs', 0)):.3f} "
                    f"CFOest={float(fm.get('estimated_cfo_hz', 0)):.0f}Hz | "
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
                    data, _ = hard_sock.recvfrom(16384)
                    got_udp = True
                    last_udp = time.time()
                    stats["udp_bytes"] += len(data)
                    incoming = "".join("1" if byte else "0" for byte in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if soft_bound:
                soft_new = _recv_soft_floats(soft_sock, stats)
                if soft_new:
                    got_udp = True
                    last_udp = time.time()
                    soft_buffer.extend(soft_new)

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]
            if len(soft_buffer) > SOFT_BUFFER_MAX:
                soft_buffer = soft_buffer[-SOFT_BUFFER_MAX:]

            new_payloads: list[bytes] = []
            if soft_bound and ENABLE_SOFT_SYNC and soft_buffer:
                soft_buffer, soft_payloads = extract_air_payloads_soft(
                    soft_buffer, stats, node.auto.access_max
                )
                new_payloads.extend(soft_payloads)
            else:
                bit_buffer, hard_payloads = extract_air_payloads_hard(
                    bit_buffer, stats, node.auto.access_max
                )
                new_payloads.extend(hard_payloads)

            # soft 优先时仍可用硬路径作补充（避免 soft 丢包）；去重靠上层 dedupe
            if soft_bound and ENABLE_SOFT_SYNC and bit_buffer:
                bit_buffer, hard_extra = extract_air_payloads_hard(
                    bit_buffer, stats, node.auto.access_max
                )
                new_payloads.extend(hard_extra)

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
        hard_sock.close()
        soft_sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
