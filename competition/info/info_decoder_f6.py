#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
info_decoder_f6.py —— 近最优信息波解码器（给无线电小白的说明）
================================================================

【一句话】能“看见”收音机前端是否削顶，并尽量在软域对齐包头，再严格 CRC 发布。

【相对 f5 多了什么】
  1) GNU Radio 里装了探头：报告 IQ 峰值/有效值、鉴频均值（粗估频偏）
     → 连续微调增益（AGC），小步调通常不清空比特缓冲
  2) 除了硬 0/1（UDP 14346），还有 soft 浮点符号（UDP 14347）
     → 用相关运算找 Access，弱信号下比单纯数汉明距离更稳
  3) 物理层带匹配滤波 + 慢偏置扣除（不是会毁掉 Header 长 0 的短 DC）
  4) 仍可用离散档大步换 FIR/带宽作兜底；IQ 可录波离线复盘

【千万记住】
  - soft 只帮你“找信封口”；Payload/业务仍硬判 + CRC，绝不软猜发布。
  - Header 仍必须精确 00 0F 00 0F。
  - 不要与 f3/f4/f5 同时开同一 Pluto/端口。

【怎么启动】
  python3 competition/info/info_decoder_f6.py
  就这一条命令，不需要任何环境变量。它会自动拉起 tx_radio6.py。

【要调参改哪里】
  - 射频档位表 + AGC 上下限 + 开机拓扑 → tx_radio6_tunes.py 顶部的调参面板
  - 软相关门槛 / 路径切换 / 协议 / 运行方式 → 本文件下面的调参面板
  - 慢偏置系数 / IQ 录波 → tx_radio6.py 的调参面板
  - 背景与调试流程 → competition/docs/信息波f6近最优接收设计.md
"""

from __future__ import annotations

from collections import deque
import datetime
import math
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
    BOOT_PROFILE,
    IQ_CLIP_FRACTION,
    IQ_CLIP_MAX_ABS,
    IQ_CLIP_RMS,
    IQ_TARGET_RMS_HIGH,
    IQ_TARGET_RMS_LOW,
    IQ_WEAK_RMS,
    RUNTIME_TUNES,
    SOFT_UDP_PORT,
)


# =============================================================================
# ★★★★★ 调参面板：现场优先只改这里 ★★★★★
#
# 直接 `python3 competition/info/info_decoder_f6.py` 启动，不需要任何环境变量。
# 射频档位与 AGC 上下限不在本文件，在 tx_radio6_tunes.py。
# =============================================================================

# 我方阵营。也可以让 /team 话题在运行中改（0=红, 1=蓝），这里只是开机默认值。
MY_CAMP = "RED"

# 与 tx_radio6.py 之间的本机管道。硬比特每个 UDP 字节 = 1 bit；
# 软符号端口在 tx_radio6_tunes.SOFT_UDP_PORT（收发两端必须一致，所以放那边）。
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_URL = "http://127.0.0.1:8081"

# 是否允许运行中自动 AGC / 换 RF 档。
# 改成 False 就变成"固定档接收"，用来和自动模式做对照实验。
ENABLE_AUTO = True

# 是否优先用软符号做包头同步。
# 改成 False 就只走硬比特路径（等价于 f5 的做法），用来判断软相关有没有帮上忙。
ENABLE_SOFT_SYNC = True

# 软相关门槛，取值是**归一化相关系数**（0~1），与信号幅度无关：
#   ρ = <soft, 模板> / (|soft| · |模板|)，1.0 表示完美对齐，纯噪声在 0 附近。
# 调法：
#   日志里 softAC 一直是 0   → 调低到 0.45~0.50（更容易抓包头，但假同步变多）
#   softAC 很多但 Header 全废 → 调高到 0.70~0.75（更严，减少假同步）
# 别低于 0.4：64 个符号的纯噪声相关标准差约 0.125，再低就淹在误报里了。
SOFT_CORR_MIN = 0.60

# 软/硬路径互切前要连续多久颗粒无收（秒）。
# 两条路解的是同一串符号，任何时刻只能有一条供货，详见主循环里的说明。
# 觉得它切得太勤就调大。
PATH_FALLBACK_SILENCE_S = 5.0

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

# 看门狗：True 时本程序会自动拉起 tx_radio6.py 并在 UDP 断流时重启它。
# 想手动分两个终端跑（比如要盯频谱窗），改成 False，然后自己先跑 tx_radio6.py。
ENABLE_GRC_WATCHDOG = True
GRC_SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "tx_radio6.py"
)

UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.5

# 把收到的原始 0/1 存成文本，事后复盘用。会增加磁盘 I/O。
# 想复盘射频而不是比特，用 tx_radio6.py 里的 IQ_RECORD_PATH。
RECORD_STREAM = False
RECORD_PREFIX = "info_f6_record"

SELF_TEST_ON_START = True
STAT_INTERVAL_S = 2.0
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 240_000
SOFT_BUFFER_MAX = 240_000
SERIAL_BUFFER_MAX = 16_384

# ---- 自动换档 / AGC 的"脾气"参数（单位：秒或计数，均相对 2s 统计窗）----
# 觉得它动作太频繁 → 调大；觉得反应太慢 → 调小，但别小于统计窗 2s，
# 否则一窗数据还没攒够就乱切。
AUTO_COOLDOWN_S = 6.0  # 两次大换档最短间隔
AUTO_EXPLORE_SILENCE_S = 4.0  # 无帧多久开始探索
AUTO_LOCK_GOOD_WINDOWS = 3  # 连续几个好窗口就锁定当前档
AUTO_RELOCK_SILENCE_S = 10.0  # 锁定后再无帧多久重新探索
AGC_COOLDOWN_S = 2.0  # 两次小步增益调整的最短间隔
# =============================================================================

# 当前开机拓扑名，只用于日志与"是否允许自动"的判断；值在 tx_radio6_tunes.py 改。
F6_PROFILE = BOOT_PROFILE

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


def _is_clipping(iq: dict) -> bool:
    """
    判定“前端真的被推饱和了”。

    必须同时满足：到轨样本占比达到门槛（不是偶发单点），并且峰值或有效值也
    确实顶到高位。旧版只看 max_abs 一个瞬时峰值，一次脉冲干扰就会误判成削顶，
    然后把增益一路降到 25dB，弱信息波直接消失——这正是 f6 收不到东西的原因之一。
    """
    if int(iq.get("count", 0) or 0) <= 0:
        return False
    clip_frac = float(iq.get("clip_frac", 0.0) or 0.0)
    max_axis = float(iq.get("max_axis", iq.get("max_abs", 0.0)) or 0.0)
    rms = float(iq.get("rms", 0.0) or 0.0)
    if clip_frac >= IQ_CLIP_FRACTION and max_axis >= IQ_CLIP_MAX_ABS:
        return True
    return rms >= IQ_CLIP_RMS


def _headroom_gain_delta(iq: dict) -> float:
    """
    根据 IQ 有效值与目标区间的差距，算出该加/该减多少 dB。

    目标是把 RMS 拉进 [LOW, HIGH]。落在区间内返回 0；偏离越远步子越大，
    但单次不超过 3 个 AGC_STEP，避免一步跨过头来回振荡。
    """
    count = int(iq.get("count", 0) or 0)
    if count <= 0:
        return 0.0
    rms = float(iq.get("rms", 0.0) or 0.0)
    if rms <= 0.0:
        return AGC_STEP_DB * 3.0
    if rms < IQ_TARGET_RMS_LOW:
        target = IQ_TARGET_RMS_LOW
    elif rms > IQ_TARGET_RMS_HIGH:
        target = IQ_TARGET_RMS_HIGH
    else:
        return 0.0
    delta_db = 20.0 * math.log10(target / rms)
    limit = AGC_STEP_DB * 3.0
    return max(-limit, min(limit, delta_db))


class AdaptiveController:
    """
    f6 自适应大脑：探头真值 + 连续 AGC + 离散档兜底。

    和 f5 最大差别：
      - 先问 GNU Radio“IQ 是不是顶平了？”再决定降不降增益
      - 小步 AGC（apply_agc_gain）默认不清空 bit/soft 缓冲
      - 只有大改 RF 带宽/FIR 时才 flush（旧比特不可信）
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
        self.last_agc_ts = 0.0
        self.last_action = "init"
        self.switches = 0
        self.agc_gain = float(RUNTIME_TUNES[start_tune]["rx_gain_db"])
        self.last_iq = {"count": 0, "max_abs": 0.0, "rms": 0.0, "clip_frac": 0.0}
        self.last_fm = {"count": 0, "estimated_cfo_hz": 0.0, "mean": 0.0}
        self.last_soft = {"count": 0, "rms": 0.0}

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
        """
        把三个探头都读空。

        注意 soft 探头也必须读：GNU Radio 侧的 vector_sink 只有被读走才会
        清空，漏读的那一个会以约 200kB/s 的速度无限吃内存，跑几十分钟就能把
        接收机拖垮。之前只读了 IQ/FM 两个，soft 探头一直在涨。
        """
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
        try:
            self.last_soft = dict(node.grc_rpc.get_soft_stats())
        except Exception:
            pass

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
        iq_count = int(iq.get("count", 0) or 0)
        rms = float(iq.get("rms", 0.0) or 0.0)
        clipping = _is_clipping(iq)
        agc_ready = now - self.last_agc_ts >= AGC_COOLDOWN_S

        if not self._rf_auto_enabled():
            self.last_action = "auto-disabled"
            return

        # 连续 AGC：只有探头确认持续削顶才降增益（不清缓冲）
        if iq_count > 0 and agc_ready and clipping:
            self._apply_agc(node, self.agc_gain - AGC_STEP_DB, now, "clip")
            return

        # 没削顶就把工作点往目标 RMS 区间推。现场实测原始 IQ 只有 -55dBFS，
        # 动态范围白白空着，这一步才是弱信号真正需要的“加曝光”。
        headroom = _headroom_gain_delta(iq)
        if iq_count > 0 and agc_ready and abs(headroom) >= 0.5:
            reason = "headroom-up" if headroom > 0 else "headroom-down"
            if self._apply_agc(node, self.agc_gain + headroom, now, reason):
                return

        if frames > 0:
            self.good_windows += 1
            if self.good_windows >= AUTO_LOCK_GOOD_WINDOWS:
                self.state = "locked"
            self.last_action = (
                f"hold:{self.tune_name}:g={self.agc_gain:.1f}:frames={frames}"
            )
            return

        self.good_windows = 0

        if self.state == "locked":
            if silence_s < AUTO_RELOCK_SILENCE_S:
                self.last_action = f"locked-wait:{silence_s:.1f}s"
                return
            self.state = "explore"

        if now - self.last_switch_ts < AUTO_COOLDOWN_S:
            self.last_action = "cooldown"
            return

        if silence_s < AUTO_EXPLORE_SILENCE_S:
            self.last_action = f"observe:{silence_s:.1f}s"
            return

        # 探头确认持续削顶且 AGC 已经降到底 → 才动用 desense 档（FIR/BW 也收）
        if clipping and self.agc_gain <= AGC_GAIN_MIN_DB + AGC_STEP_DB:
            if self.tune_name != "desense":
                self._apply_tune(node, "desense", now, "probe-clip", flush=True)
                return
            self.last_action = "clip-floor"
            return

        if ac >= 4 and header_ok == 0 and header_fail >= max(1, ac // 2):
            target = "wide_fir" if self.tune_name != "wide_fir" else "weak_boost"
            self._apply_tune(node, target, now, "header-starve", flush=True)
            return

        if header_ok >= 2 and frames == 0:
            target = self._next_gain_step()
            if target:
                self._apply_tune(node, target, now, "payload-starve", flush=True)
                return

        # 探头说“天线上几乎什么都没有”，而且一个 Access 都没蹭到：大步加增益。
        if iq_count > 0 and rms < IQ_WEAK_RMS and ac == 0 and agc_ready:
            if self._apply_agc(
                node, self.agc_gain + AGC_STEP_DB * 2.0, now, "weak-noac"
            ):
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


_ACCESS_TEMPLATE_NORM = math.sqrt(64.0)


def _soft_corr_at(soft: list[float], offset: int) -> float:
    """
    归一化相关系数 ρ = <soft, 模板> / (|soft| · |模板|)，范围 -1~1。

    为什么必须除以 |soft|：鉴频器输出的幅度随增益、信噪比、离散档一起漂，
    裸点积因此没有绝对意义。归一化之后 ρ 只反映“波形长得像不像”，
    +1 是完全对齐，-1 是整包反相，噪声段自然落在 0 附近。
    """
    dot = 0.0
    energy = 0.0
    for i, pm in enumerate(ACCESS_PM):
        sample = soft[offset + i]
        dot += sample * pm
        energy += sample * sample
    if energy <= 0.0:
        return 0.0
    return dot / (math.sqrt(energy) * _ACCESS_TEMPLATE_NORM)


def find_access_soft(
    soft: list[float],
    *,
    corr_min: float,
    access_max: int,
) -> Optional[tuple[int, bool, float, int]]:
    """
    在软符号流上找 Access（从左到右第一个过阈值的位置）。

    白话：把 Access 的 0/1 看成 -1/+1 模板，和 soft 波形比“形状相似度”。
    ρ 接近 +1 表示对齐，接近 -1 表示整包反相（GFSK 极性翻转）。
    返回 (offset, inverted, ρ 的绝对值, 由 sign(soft) 估的硬汉明距离)。
    """
    if len(soft) < 64:
        return None
    last = len(soft) - 64
    thr = max(0.0, min(1.0, corr_min))
    for offset in range(last + 1):
        rho = _soft_corr_at(soft, offset)
        inv = rho < 0.0
        score = -rho if inv else rho
        if score < thr:
            continue
        hard_bits = "".join(
            "1" if soft[offset + i] >= 0 else "0" for i in range(64)
        )
        if inv:
            hard_bits = base.invert_bits(hard_bits)
        dist = (int(hard_bits, 2) ^ AC_NORMAL_INT).bit_count()
        return offset, inv, score, dist
    return None


def soft_to_hard_bits(soft: list[float], inverted: bool) -> str:
    bits = "".join("1" if sample >= 0 else "0" for sample in soft)
    return base.invert_bits(bits) if inverted else bits


def extract_air_payloads_soft(
    soft: list[float], stats: dict, access_max: int
) -> tuple[list[float], list[bytes]]:
    """
    软域拆空口包：相关找 Access → sign() 得到硬比特 → Header 仍硬匹配。

    soft 再聪明也只负责同步；Header/CRC 铁律与 f3/f4 相同。软路径唯一的
    放行条件就是归一化相关系数过阈值，误判全部交给严格 Header 兜住。
    """
    payloads: list[bytes] = []
    while len(soft) >= AIR_FRAME_SYMBOLS:
        found = find_access_soft(
            soft, corr_min=SOFT_CORR_MIN, access_max=access_max
        )
        if found is None:
            # 整段都没找到 Access。Access 只有 64 个符号，所以还有机会拼出
            # 包头的只剩最后 63 个符号，保留它们就够了。
            #
            # 这里原来保留 AIR_FRAME_SYMBOLS-1（215）个，是压垮 f6 的性能 bug：
            # 主循环每毫秒才进来约 21 个新符号，却要把已经判过、确定不是包头的
            # 215 个符号连同新符号一起重新做一遍归一化相关，白做约 8 倍的功。
            # 实测这一条就让软路径吃满 1.06 倍实时，UDP 缓冲直接溢出，
            # 表现就是"程序像卡住、所有 Hz 全为 0"。
            soft = soft[-(AIR_ACCESS_LEN * 8 - 1) :]
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
    """硬比特路径（与 f4 相同思路）：soft 端口挂了也能单独工作。"""
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
    """
    纯内存自检：硬切片 + 软相关 + 坏 Header 必丢 + Access 自适应收紧 + CRC。
    不连电台，也不往正式 ROS 话题灌假数据。
    """
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

    # 回归用例：没找到 Access 时只能保留最后 63 个符号。
    # 保留多了功能上照样"正确"，但每次轮询都会把已经判过的符号重新做一遍
    # 归一化相关，白做约 8 倍的功，软路径直接吃满实时、UDP 溢出、一帧不出。
    # 这种性能坑不会报错，只能靠断言守住。
    noise_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    noise_soft = [
        0.5 if (index * 7919) % 3 == 0 else -0.5
        for index in range(AIR_FRAME_SYMBOLS * 3)
    ]
    noise_remain, noise_payloads = extract_air_payloads_soft(
        noise_soft, noise_stats, ACCESS_MAX_HAMMING_LOOSE
    )
    retention_ok = not noise_payloads and len(noise_remain) < AIR_ACCESS_LEN * 8

    # 回归用例：软/硬两条路径同时供货一定会毁掉裁判帧。
    # 这里刻意复现旧版的错误合并方式，断言它确实解不出 15 帧，
    # 以后谁再把两条路径接回一起，自检就会立刻报错。
    interleaved = bytearray()
    for soft_payload, hard_payload in zip(soft_payloads, payloads):
        interleaved.extend(soft_payload)
        interleaved.extend(hard_payload)
    dup_stats = _fresh_stats(ACCESS_MAX_HAMMING_LOOSE)
    dup_frames = drain_strict_frames(bytearray(interleaved), dup_stats)
    single_path_ok = len(dup_frames) < 15

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
    clip_ok = (
        _is_clipping({"count": 1000, "clip_frac": 0.05, "max_axis": 0.99, "rms": 0.4})
        and not _is_clipping(
            {"count": 1000, "clip_frac": 0.0, "max_axis": 0.99, "rms": 0.02}
        )
        and _headroom_gain_delta({"count": 1000, "rms": 0.01}) > 0
        and _headroom_gain_delta({"count": 1000, "rms": 0.15}) == 0.0
    )

    ok = (
        hard_ok
        and soft_ok
        and header_ok
        and frame_ok
        and access_adapt_ok
        and single_path_ok
        and retention_ok
        and clip_ok
    )
    node.get_logger().info(
        f"自检 硬切片={'通过' if hard_ok else '失败'} "
        f"软相关={'通过' if soft_ok else '失败'} "
        f"严格Header={'通过' if header_ok else '失败'} "
        f"15帧={'通过' if frame_ok else '失败'} "
        f"Access自适应={'通过' if access_adapt_ok else '失败'} "
        f"单路径守恒={'通过' if single_path_ok else '失败'} "
        f"软缓冲保留={'通过' if retention_ok else '失败'} "
        f"削顶判定={'通过' if clip_ok else '失败'}"
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
    """
    主循环：
      收硬比特 UDP +（可选）软符号 UDP
      → 优先软相关拆包，硬路径补充
      → 拼裁判帧 → CRC → ROS
      → 每 2s 用探头/统计做 AGC 或换档
    """
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("f6 离线自检未通过，请勿直接用于赛场！")

    # tx_radio6.py 和本文件读的是同一个 tx_radio6_tunes.BOOT_PROFILE，
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
    # 同一时刻只有一条拆包路径在供货，见下方主循环里的说明。
    active_path = "soft" if (soft_bound and ENABLE_SOFT_SYNC) else "hard"
    last_path_payload = time.time()
    last_path_switch = time.time()

    node.get_logger().info(
        f"监听 hard={UDP_PORT} soft={SOFT_UDP_PORT if soft_bound else 'off'} | "
        f"GRC={GRC_SCRIPT_PATH} | profile={F6_PROFILE} | 拆包路径={active_path}"
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
                last_path_payload = now

            if ENABLE_GRC_WATCHDOG and now - last_udp > UDP_WATCHDOG_S:
                node.get_logger().warn("UDP 断流，重启 tx_radio6…")
                base.restart_grc(node)
                last_udp = time.time()
                bit_buffer = ""
                soft_buffer = []
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()
                last_path_payload = last_udp
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
                    f"path={active_path} {elapsed:.1f}s] {rates} | "
                    f"AC0..{access_cap}={access_hist} softAC={stats['ac_soft_hits']} "
                    f"corr={stats['last_soft_corr']:.2f} "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16={stats['crc8_fail']}/{stats['crc16_fail']} "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"无帧={silence:.1f}s | "
                    f"IQrms={float(iq.get('rms', 0)):.3f} "
                    f"IQmax={float(iq.get('max_abs', 0)):.3f} "
                    f"clip={float(iq.get('clip_frac', 0)):.4f} "
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

            # 软路径和硬路径解的是**同一串符号**（硬比特就是软符号取符号位），
            # 两条路都往 serial_buffer 里灌，就会得到 P1软 P1硬 P2软 P2硬…，
            # 15 字节的 Payload 被整段重复插入，跨 Payload 的裁判帧必然错位、
            # CRC16 永远算不过——这就是 f6 一帧都收不到的根因。
            # 因此任何时刻只允许一条路径供货，另一条清空缓冲待命。
            new_payloads: list[bytes] = []
            if active_path == "soft":
                bit_buffer = ""
                soft_buffer, new_payloads = extract_air_payloads_soft(
                    soft_buffer, stats, node.auto.access_max
                )
                if new_payloads:
                    last_path_payload = now
                elif (
                    now - last_path_payload > PATH_FALLBACK_SILENCE_S
                    and now - last_path_switch > PATH_FALLBACK_SILENCE_S
                ):
                    active_path = "hard"
                    last_path_switch = now
                    last_path_payload = now
                    soft_buffer = []
                    serial_buffer.clear()
                    packet_window.clear()
                    node.get_logger().info("[f6] 软路径无产出 → 切硬比特路径")
            else:
                soft_buffer = []
                bit_buffer, new_payloads = extract_air_payloads_hard(
                    bit_buffer, stats, node.auto.access_max
                )
                if new_payloads:
                    last_path_payload = now
                elif (
                    soft_bound
                    and ENABLE_SOFT_SYNC
                    and now - last_path_payload > PATH_FALLBACK_SILENCE_S
                    and now - last_path_switch > PATH_FALLBACK_SILENCE_S
                ):
                    active_path = "soft"
                    last_path_switch = now
                    last_path_payload = now
                    bit_buffer = ""
                    serial_buffer.clear()
                    packet_window.clear()
                    node.get_logger().info("[f6] 硬路径无产出 → 切软相关路径")

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
