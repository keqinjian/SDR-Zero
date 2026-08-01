#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
f6 IQ 离线回放（SPS=47）。

读取 Pluto 复数录波（interleaved float32 .c64 / .fc32），做与在线相近的简化解调：
  QuadDemod → 慢偏置 → 高斯匹配滤波 → 每 SPS 抽样 → 软 Access 统计。

不依赖 GNU Radio；用于区分“前端已坏”与“协议层丢包”。
用法：
  python3 competition/info/info_f6_iq_replay.py recording.c64
  INFO_F6_IQ_REPLAY_MAX=2000000 python3 ... recording.c64
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
from pathlib import Path

# 保证可从任意 cwd 导入同目录模块
sys.path.insert(0, str(Path(__file__).resolve().parent))

# 离线工具不依赖真实 ROS；为导入 f6 提供最小桩。
import types

if "rclpy" not in sys.modules:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
if "std_msgs" not in sys.modules:
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Int8 = type("Int8", (), {})
    std_msgs_msg.String = type("String", (), {})
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

import info_decoder_f6 as f6


SPS = 47
SENSITIVITY = 1.5628
BT = 0.35
SAMPLE_RATE = 1_000_000
BIAS_ALPHA = float(os.environ.get("INFO_F6_BIAS_ALPHA", "1e-5"))


def read_c64(path: Path, max_samples: int | None) -> list[complex]:
    raw = path.read_bytes()
    usable = len(raw) - (len(raw) % 8)
    count = usable // 8
    if max_samples is not None:
        count = min(count, max_samples)
    values = struct.unpack(f"<{count * 2}f", raw[: count * 8])
    return [complex(values[i], values[i + 1]) for i in range(0, len(values), 2)]


def gaussian_taps(sps: int, bt: float, span: int) -> list[float]:
    """近似 firdes.gaussian：BT 高斯脉冲，归一化能量。"""
    ntaps = span * sps + 1
    # GNU Radio gaussian: std = sps * sqrt(2*ln2) / (pi*BT) 一类定义的离散采样
    # 这里用工程近似，足够做离线对比趋势。
    sigma = 0.399 * sps / max(bt, 1e-6)
    mid = (ntaps - 1) / 2.0
    taps = []
    for i in range(ntaps):
        x = i - mid
        taps.append(math.exp(-0.5 * (x / sigma) ** 2))
    energy = math.sqrt(sum(t * t for t in taps)) or 1.0
    return [t / energy for t in taps]


def fir_filter(x: list[float], taps: list[float]) -> list[float]:
    n = len(taps)
    if len(x) < n:
        return []
    out = []
    for i in range(n - 1, len(x)):
        acc = 0.0
        base_i = i - n + 1
        for k, t in enumerate(taps):
            acc += x[base_i + k] * t
        out.append(acc)
    return out


def quadrature_demod(iq: list[complex], sensitivity: float) -> list[float]:
    if len(iq) < 2:
        return []
    gain = 1.0 / sensitivity
    out = []
    prev = iq[0]
    for sample in iq[1:]:
        # angle(conj(prev)*sample)
        prod = prev.conjugate() * sample
        out.append(gain * math.atan2(prod.imag, prod.real))
        prev = sample
    return out


def slow_bias_remove(x: list[float], alpha: float) -> list[float]:
    mean = 0.0
    out = []
    for value in x:
        mean = (1.0 - alpha) * mean + alpha * value
        out.append(value - mean)
    return out


def decimate_symbols(x: list[float], sps: int) -> list[float]:
    if sps <= 0:
        return []
    # 取每符号中点附近
    offset = sps // 2
    return [x[i] for i in range(offset, len(x), sps)]


def main() -> int:
    parser = argparse.ArgumentParser(description="f6 IQ 离线回放（SPS=47）")
    parser.add_argument("iq_path", type=Path, help=".c64/.fc32 interleaved float32 IQ")
    parser.add_argument(
        "--max-samples",
        type=int,
        default=int(os.environ.get("INFO_F6_IQ_REPLAY_MAX", "0") or 0) or None,
        help="最多读取的复数采样数；0 表示全部",
    )
    args = parser.parse_args()
    if not args.iq_path.is_file():
        print(f"文件不存在: {args.iq_path}", file=sys.stderr)
        return 1

    iq = read_c64(args.iq_path, args.max_samples)
    if not iq:
        print("IQ 为空", file=sys.stderr)
        return 1

    abs_vals = [abs(s) for s in iq]
    max_abs = max(abs_vals)
    rms = math.sqrt(sum(a * a for a in abs_vals) / len(abs_vals))
    print(
        f"IQ samples={len(iq)} max_abs={max_abs:.4f} rms={rms:.4f} "
        f"{'(疑似削顶)' if max_abs >= 0.92 else ''}"
    )

    fm = quadrature_demod(iq, SENSITIVITY)
    fm = slow_bias_remove(fm, BIAS_ALPHA)
    taps = gaussian_taps(SPS, BT, span=4)
    filtered = fir_filter(fm, taps)
    soft = decimate_symbols(filtered, SPS)
    print(f"soft symbols={len(soft)} taps={len(taps)} bias_alpha={BIAS_ALPHA}")

    stats = f6._fresh_stats(f6.ACCESS_MAX_HAMMING_LOOSE)
    remain, payloads = f6.extract_air_payloads_soft(
        soft, stats, f6.ACCESS_MAX_HAMMING_LOOSE
    )
    extracted = b"".join(payloads)
    print(
        f"softAC={stats['ac_soft_hits']} HeaderOK/FAIL="
        f"{stats['header_ok']}/{stats['header_fail']} "
        f"payloads={len(payloads)} remain_soft={len(remain)} "
        f"last_corr={stats['last_soft_corr']:.1f}"
    )

    if extracted:
        frames = f6.find_valid_frames(extracted)
        print(f"window CRC frames={len(frames)}")
        for frame in frames[:8]:
            cmd = struct_unpack_cmd(frame)
            print(f"  frame cmd=0x{cmd:04X} len={len(frame)}")
    else:
        print("未抽出空口 Payload；检查增益/频点/是否录到信息波。")
    return 0


def struct_unpack_cmd(frame: bytes) -> int:
    import struct

    if len(frame) < 7:
        return 0
    return struct.unpack_from("<H", frame, 5)[0]


if __name__ == "__main__":
    raise SystemExit(main())
