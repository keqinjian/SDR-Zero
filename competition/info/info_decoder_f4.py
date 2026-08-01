#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
info_decoder_f4.py —— 弱信号信息波解码器（给无线电小白的说明）
================================================================

【一句话】在强干扰、信息波很弱（约 -60 dB 量级）时，尽量仍能解出位置/血量等。

【和 f3 的分工】
  - 协议层（Access / Header / CRC / ROS）与 f3 同一套铁律，不能改歪官方格式。
  - 物理层换成 tx_radio4：把黑盒 GFSK Demod 拆开，方便弱信号调时钟恢复。
  - Access 默认可错 4 bit（比 f3 的 2 更宽松）；Header 仍必须精确 00 0F 00 0F。

【物理层在干什么（白话）】
  天线 IQ → 滤波 → 鉴频（听音调高低）→ 可选平滑 → 时钟对齐 → 判 0/1
  → UDP 送到本程序。官方 SPS=47、Sens=1.5628 绝不能改错。

【重要提醒】
  - 判决域短 DC blocker 会把 Header 里“连续很多个 0”当成直流滤掉，f4 正式档默认关。
  - 自检只在内存里跑，不会往正式 ROS 话题灌假数据。
  - 不要和 f3 同时开：共用 Pluto、UDP 14346、RPC 8081。

【怎么启动】
  INFO_F4_PROFILE=weak_antijam python3 competition/info/info_decoder_f4.py
  详见 competition/docs/信息波f4弱信号调试指南.md
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


# =============================================================================
# ★★★★★ 现场优先只改这里 ★★★★★
# =============================================================================
MY_CAMP = "RED"  # /team: 0=红, 1=蓝
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_URL = "http://127.0.0.1:8081"

# weak_antijam = 弱信号抗干扰档；baseline = 接近 f3 基线，用来回归
F4_PROFILE = os.environ.get("INFO_F4_PROFILE", "weak_antijam").strip().lower()
# Access 容错上限（环境变量可改回 2，更像 f3）
ACCESS_MAX_HAMMING = int(os.environ.get("INFO_F4_ACCESS_HAMMING", "4"))
# Header 不允许花码：官方两份长度字段必须一致，且固定为 15
HEADER_MAX_HAMMING = 0

ENABLE_PACKET_WINDOW_RECOVERY = True  # 与 f3 相同的窗口 CRC 复扫
PACKET_WINDOW_PAYLOADS = 8
FRAME_DEDUP_TTL_S = 2.0

ENABLE_GRC_WATCHDOG = os.environ.get("RM_ENABLE_GRC_WATCHDOG", "1") not in (
    "0", "false", "False", "no", "NO",
)
GRC_SCRIPT_PATH = os.environ.get(
    "INFO_GRC_SCRIPT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "tx_radio4.py"),
)
UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.0
RECORD_STREAM = os.environ.get("INFO_F4_RECORD", "0") in (
    "1", "true", "True", "yes", "YES",
)
RECORD_PREFIX = "info_f4_record"
SELF_TEST_ON_START = True
STAT_INTERVAL_S = 2.0  # 比 f3 更勤：弱信号调参需要更快反馈
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 50_000
SERIAL_BUFFER_MAX = 8_192
# =============================================================================

# 协议常量全部复用 f1（已对齐 V2）
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
CMD_DATA_LEN = base.CMD_DATA_LEN

# 看门狗拉起 tx_radio4，而不是 f3 的 tx_radio2
base.ENABLE_GRC_WATCHDOG = ENABLE_GRC_WATCHDOG
base.GRC_SCRIPT_PATH = GRC_SCRIPT_PATH
base.UDP_WATCHDOG_S = UDP_WATCHDOG_S
base.GRC_BOOT_WAIT_S = GRC_BOOT_WAIT_S
base.RPC_URL = RPC_URL

# 组帧 / 发布逻辑与 f3 完全一致，避免两套实现漂开
find_valid_frames = f3.find_valid_frames
drain_strict_frames = f3.drain_strict_frames
handle_valid_frame = f3.handle_valid_frame
f3.FRAME_DEDUP_TTL_S = FRAME_DEDUP_TTL_S


class InfoDecoderNode(base.Node):
    """ROS 接口与 f3 相同（话题名不变），节点名改为 info_decoder_f4。"""

    def __init__(self) -> None:
        super().__init__("info_decoder_f4")
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
            f"信息波 f4 启动 | 阵营={self.ally_camp} profile={F4_PROFILE} | "
            f"Access≤{ACCESS_MAX_HAMMING}bit Header=严格匹配 | "
            "正式话题仅发布 CRC 有效帧"
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
        self.get_logger().info(
            f"tx_radio4 已切至 {freq/1e6:.3f}MHz；profile={F4_PROFILE}"
        )
        return True


def _fresh_stats() -> dict:
    """每个统计窗口清零用的计数器（日志里 AC0..N、Header OK/FAIL 等）。"""
    return {
        "ac_hits": 0,
        "ac_hits_inverted": 0,
        "access_hamming": {i: 0 for i in range(ACCESS_MAX_HAMMING + 1)},
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


def _find_access(bit_buffer: str) -> Optional[tuple[int, bool, int]]:
    """
    在 0/1 海里找 64 位 Access（信息波暗号）。

    返回 (起点, 是否反相, 汉明距离)。
    与 f3 相同算法，只是默认允许差到 4 位——弱信号下更容易“看见”包头，
    但假同步增多的风险靠后面的严格 Header + CRC 挡住。
    """
    if len(bit_buffer) < 64:
        return None
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
            window = ((window << 1) & ACCESS_MASK) | (
                bit_buffer[offset + 64] == "1"
            )
    return None


def _header_hamming(header_bits: str) -> int:
    """Header 与官方 00 0F 00 0F 差几位（f4 要求必须为 0）。"""
    return base.hamming(header_bits, HEADER_OFFICIAL_BITS)


def extract_air_payloads(
    bit_buffer: str, stats: dict
) -> tuple[str, list[bytes]]:
    """
    从比特流拆出空口 Payload（每片 15 字节）。

    拆快递口诀：
      1) Access 可容错 → 找到信封口
      2) 反相则先翻正
      3) Header 必须一字不差；错了只前进 1 bit（别连坐丢掉后面真包）
      4) 只有真信封才整包吃掉 216 bit
    注意：这里只负责“空口片”；拼成 0x0A01～0x0A05 仍要过 CRC。
    """
    payloads: list[bytes] = []
    while len(bit_buffer) >= AIR_FRAME_BITS:
        found = _find_access(bit_buffer)
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
    纯内存协议自检（不需要 Pluto）。

    覆盖：连续 420B 切片、Access 4bit 容错、坏 Header 必丢、CRC 双路径去重。
    故意不调用业务 parser/publisher，避免假数据污染正式 ROS 话题。
    """
    node.get_logger().info("==== f4 纯内存自检开始 ====")
    rounds = b"".join(_build_test_round(index * 5) for index in range(3))
    if len(rounds) != 420:
        node.get_logger().error(f"连续三轮长度错误: {len(rounds)}B，应为420B")
        return False

    clean_bits = _wrap_continuous_air_bits(rounds)
    # 第一包 Access 损伤4bit，验证同步容错不改变 Payload。
    access_damaged = _flip_bits(clean_bits, (3, 20, 41, 55))

    stats = _fresh_stats()
    remain, payloads = extract_air_payloads(access_damaged, stats)
    extracted = b"".join(payloads)
    access_ok = len(payloads) == 28 and not remain and extracted == rounds

    # Header 任一 bit 损坏都必须丢弃该空口包，不能污染连续字节流。
    second_header = AIR_FRAME_BITS + 64
    header_damaged = _flip_bits(clean_bits, (second_header,))
    header_stats = _fresh_stats()
    header_remain, header_payloads = extract_air_payloads(
        header_damaged, header_stats
    )
    expected_without_second_payload = rounds[:15] + rounds[30:]
    strict_header_ok = (
        len(header_payloads) == 27
        and not header_remain
        and b"".join(header_payloads) == expected_without_second_payload
        and header_stats["header_fail"] == 1
    )

    strict_stats = _fresh_stats()
    strict_frames = drain_strict_frames(bytearray(extracted), strict_stats)
    window_frames = find_valid_frames(extracted)
    cmd_ids = [struct.unpack_from("<H", frame, 5)[0] for frame in strict_frames]
    frame_ok = (
        len(strict_frames) == 15
        and len(window_frames) == 15
        and cmd_ids.count(0x0A05) == 3
    )
    strict_keys = {
        (frame[3], struct.unpack_from("<H", frame, 5)[0], frame[-2:])
        for frame in strict_frames
    }
    window_keys = {
        (frame[3], struct.unpack_from("<H", frame, 5)[0], frame[-2:])
        for frame in window_frames
    }
    dedupe_ok = strict_keys == window_keys and len(strict_keys) == 15
    ok = access_ok and strict_header_ok and frame_ok and dedupe_ok

    node.get_logger().info(
        f"自检 连续切片={'通过' if access_ok else '失败'} "
        f"严格Header={'通过' if strict_header_ok else '失败'} "
        f"15帧/0x0A05={'通过' if frame_ok else '失败'} "
        f"双路径去重键={'通过' if dedupe_ok else '失败'}"
    )
    node.get_logger().info(
        "==== f4 纯内存自检结束：%s ====" % ("全部通过 OK" if ok else "存在失败 FAIL")
    )
    return ok


def connect_grc(node: InfoDecoderNode) -> None:
    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
        else:
            node.get_logger().error("XMLRPC 调用失败，请先启动 tx_radio4.py")
    except Exception as exc:
        node.get_logger().error(f"无法连接 XMLRPC({RPC_URL}): {exc}")


def main() -> None:
    """主循环：收 UDP 比特 → 拆空口包 → 拼裁判帧 → CRC 后发 ROS。"""
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("f4 离线自检未通过，请勿直接用于赛场！")

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
    stats = _fresh_stats()
    last_stat = time.time()
    last_udp = time.time()
    last_valid_frame = time.time()

    node.get_logger().info(
        f"监听 {UDP_IP}:{UDP_PORT} | GRC={GRC_SCRIPT_PATH} | profile={F4_PROFILE}"
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
                node.get_logger().warn("UDP 断流，重启 tx_radio4…")
                base.restart_grc(node)
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
                access_hist = "/".join(
                    str(stats["access_hamming"].get(i, 0))
                    for i in range(ACCESS_MAX_HAMMING + 1)
                )
                header_hist = "/".join(
                    str(stats["header_hamming"].get(i, 0))
                    for i in range(HEADER_MAX_HAMMING + 1)
                )
                node.get_logger().info(
                    f"[f4 {F4_PROFILE} {elapsed:.1f}s] {rates} | "
                    f"AC0..{ACCESS_MAX_HAMMING}={access_hist} "
                    f"HDR0..{HEADER_MAX_HAMMING}={header_hist} "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16={stats['crc8_fail']}/{stats['crc16_fail']} "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"无有效帧={now-last_valid_frame:.1f}s UDP={stats['udp_bytes']}"
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
                    incoming = "".join("1" if byte else "0" for byte in data)
                    bit_buffer += incoming
                    if file_recorder is not None:
                        file_recorder.write(incoming + "\n")
            except BlockingIOError:
                pass

            if len(bit_buffer) > BIT_BUFFER_MAX:
                bit_buffer = bit_buffer[-BIT_BUFFER_MAX:]

            bit_buffer, new_payloads = extract_air_payloads(bit_buffer, stats)
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
