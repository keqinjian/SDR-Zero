#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboMaster 2026 信息波渐进抗干扰解码器 f3。

f3 不改现有 radio/info/* 接口，也不发布 CRC 失败的数据。相对 f1 首版只做：
  1) 完整 64-bit Access Code 汉明距离同步（正/反相）；
  2) 保留严格字节流解析，并增加 CRC 有效的 Payload 窗口复扫；
  3) 统一去重、按命令统计 Hz，便于联合测试逐项调参。

物理层由同目录 tx_radio2.py 提供。先用 baseline 回归，再切 balanced/strong；
不要一次把所有参数改满，否则现场无法判断是哪一项真正有效。
"""

from __future__ import annotations

from collections import deque
import datetime
import os
import socket
import struct
import time
import xmlrpc.client
from typing import Callable, Optional

import rclpy
from std_msgs.msg import Int8, String

import info_decoder_f1 as base


# =============================================================================
# ★★★★★ 解码抗干扰参数调试区（现场优先只改这里）★★★★★
# =============================================================================
MY_CAMP = "RED"  # /team: Int8 0=RED, 1=BLUE 会覆盖
UDP_IP = "127.0.0.1"
UDP_PORT = 14346
RPC_URL = "http://127.0.0.1:8081"

# 与 tx_radio2.py 使用同一个环境变量；仅用于选择物理层档位和日志标识。
ANTIJAM_PROFILE = os.environ.get("INFO_ANTIJAM_PROFILE", "balanced").strip().lower()

# 完整 64 bit Access 容错。2 表示允许任意两位花码；Header 仍必须完全正确。
ENABLE_FULL_ACCESS_HAMMING = True
ACCESS_MAX_HAMMING = 2

# CRC 有效的包窗口复扫。它不会发布 CRC 失败的“猜测数据”。
ENABLE_PACKET_WINDOW_RECOVERY = True
PACKET_WINDOW_PAYLOADS = 8       # 8×15B 足够覆盖最大 0x0A05 帧及相邻边界
FRAME_DEDUP_TTL_S = 2.0

ENABLE_GRC_WATCHDOG = os.environ.get("RM_ENABLE_GRC_WATCHDOG", "1") not in (
    "0", "false", "False", "no", "NO",
)
GRC_SCRIPT_PATH = os.environ.get(
    "INFO_GRC_SCRIPT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "tx_radio2.py"),
)
UDP_WATCHDOG_S = 2.0
GRC_BOOT_WAIT_S = 3.0
RECORD_STREAM = True
RECORD_PREFIX = "info_f3_record"
SELF_TEST_ON_START = True
STAT_INTERVAL_S = 5.0
IDLE_SLEEP_S = 0.001
BIT_BUFFER_MAX = 50_000
SERIAL_BUFFER_MAX = 8_192
# =============================================================================


# 沿用 f1 已对齐 V2 的协议、CRC 与业务解析，避免复制两套协议后逐渐漂移。
INFO_FREQ_MAP = base.INFO_FREQ_MAP
INFO_SENSITIVITY = base.INFO_SENSITIVITY
ACCESS_CODE_HEX = base.ACCESS_CODE_HEX
AC_NORMAL = base.AC_NORMAL
AC_INVERTED = base.AC_INVERTED
AC_NORMAL_INT = int(AC_NORMAL, 2)
ACCESS_MASK = (1 << 64) - 1
AIR_FRAME_BITS = base.AIR_FRAME_BITS
AIR_ACCESS_LEN = base.AIR_ACCESS_LEN
AIR_HEADER_LEN = base.AIR_HEADER_LEN
AIR_PAYLOAD_LEN = base.AIR_PAYLOAD_LEN
HEADER_OFFICIAL = base.HEADER_OFFICIAL
CMD_DATA_LEN = base.CMD_DATA_LEN

# 让复用的 f1 看门狗函数拉起 tx_radio2，而不是旧 tx_radio。
base.ENABLE_GRC_WATCHDOG = ENABLE_GRC_WATCHDOG
base.GRC_SCRIPT_PATH = GRC_SCRIPT_PATH
base.UDP_WATCHDOG_S = UDP_WATCHDOG_S
base.GRC_BOOT_WAIT_S = GRC_BOOT_WAIT_S
base.RPC_URL = RPC_URL


class InfoDecoderNode(base.Node):
    """保持 f1 的 ROS 接口，节点名改为 info_decoder_f3。"""

    def __init__(self) -> None:
        super().__init__("info_decoder_f3")
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
            f"信息波 f3 启动 | 阵营={self.ally_camp} | profile={ANTIJAM_PROFILE} | "
            f"Access=64bit/Hamming≤{ACCESS_MAX_HAMMING} | "
            f"窗口恢复={'开' if ENABLE_PACKET_WINDOW_RECOVERY else '关'}"
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
            self.grc_rpc.set_target_sens(INFO_SENSITIVITY)
        except Exception as exc:
            self.get_logger().error(f"切频/设 Sens 失败: {exc}")
            return False
        self.get_logger().info(
            f"tx_radio2 已切至 {freq/1e6:.3f} MHz；profile={ANTIJAM_PROFILE}"
        )
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
    返回 (offset, 是否反相, 汉明距离)。

    滚动 64-bit 整数只做 XOR + bit_count；反相码是正相码逐位取反，
    因而反相距离等于 64 - 正相距离。
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


def extract_air_payloads(
    bit_buffer: str, stats: dict
) -> tuple[str, list[bytes]]:
    """
    完整 Access 汉明同步 → 极性纠正 → 精确 Header 门闩 → 15B Payload。
    假同步只前进 1 bit；真包才跨过完整 216 bit。
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
        frame_bits = base.invert_bits(raw_bits) if inverted else raw_bits
        frame_bytes = base.bits_to_bytes(frame_bits)
        stats["ac_hits"] += 1
        stats["access_hamming"].setdefault(distance, 0)
        stats["access_hamming"][distance] += 1
        if inverted:
            stats["ac_hits_inverted"] += 1

        header = frame_bytes[
            AIR_ACCESS_LEN:AIR_ACCESS_LEN + AIR_HEADER_LEN
        ]
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
    if start + 9 > len(data) or data[start] != 0xA5:
        return None
    if base.calc_crc8(data[start:start + 4]) != data[start + 4]:
        return None
    data_len = struct.unpack_from("<H", data, start + 1)[0]
    frame_len = 5 + 2 + data_len + 2
    if data_len > 64 or frame_len > 80 or start + frame_len > len(data):
        return None
    frame = data[start:start + frame_len]
    got_crc16 = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    return frame if base.calc_crc16(frame[:-2]) == got_crc16 else None


def find_valid_frames(data: bytes) -> list[bytes]:
    """不修改输入，在任意 Payload 窗口里搜索完整 CRC 有效裁判帧。"""
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
    """f1 严格字节河路径：CRC 失败滑 1 字节，完整有效帧才返回。"""
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
        if base.calc_crc8(serial_buffer[:4]) != serial_buffer[4]:
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
        if base.calc_crc16(frame[:-2]) != got_crc16:
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
    node: InfoDecoderNode,
    packet_counter: dict,
    stats: dict,
    dedupe: dict,
    now: float,
) -> bool:
    """统一长度门闩、去重、业务解析；返回是否首次发布。"""
    data_len = struct.unpack_from("<H", frame, 1)[0]
    seq = frame[3]
    cmd_id = struct.unpack_from("<H", frame, 5)[0]
    crc16 = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    key = (seq, cmd_id, crc16)
    _cleanup_dedupe(dedupe, now)
    if key in dedupe:
        stats["dedup_frames"] += 1
        return False
    dedupe[key] = now

    expected = CMD_DATA_LEN.get(cmd_id)
    if expected is not None and data_len != expected:
        stats["len_mismatch"] += 1
        return False

    payload = frame[7:7 + data_len]
    parser_map: dict[int, tuple[Callable, str]] = {
        0x0A01: (base.parse_0x0A01, "0x0A01"),
        0x0A02: (base.parse_0x0A02, "0x0A02"),
        0x0A03: (base.parse_0x0A03, "0x0A03"),
        0x0A04: (base.parse_0x0A04, "0x0A04"),
        0x0A05: (base.parse_0x0A05, "0x0A05"),
    }
    item = parser_map.get(cmd_id)
    if item is None:
        stats["unknown_cmd"] += 1
        return False
    parser, counter_key = item
    parser(payload, node)
    packet_counter[counter_key] += 1
    stats["frames_ok"] += 1
    stats["strict_frames" if source == "strict" else "window_frames"] += 1
    return True


def run_self_test(node: InfoDecoderNode) -> bool:
    """覆盖 Access 0/1/2/3 位错误、反相、坏 Header、窗口扫描。"""
    node.get_logger().info("==== f3 抗干扰自检开始 ====")
    payload = struct.pack("<12H", 100, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    frame = base.build_referee_frame(0x0A01, payload, seq=7)
    clean_bits = base.wrap_stream_as_air_bits(frame)

    def corrupt_access(bits: str, positions: tuple[int, ...]) -> str:
        changed = list(bits)
        for pos in positions:
            changed[pos] = "1" if changed[pos] == "0" else "0"
        return "".join(changed)

    ok = True
    for errors, positions in (
        (0, ()),
        (1, (3,)),
        (2, (3, 41)),
    ):
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
    _, payloads_3 = extract_air_payloads(
        corrupt_access(clean_bits, (3, 20, 41)), stats
    )
    # 仅第一空口包的 Access 被破坏；后两包仍应挂上。
    reject_3 = len(payloads_3) == 2
    ok = ok and reject_3
    node.get_logger().info(f"自检 Access 3bit 拒绝：{'通过' if reject_3 else '失败'}")

    stats = _fresh_stats()
    _, inverted_payloads = extract_air_payloads(base.invert_bits(clean_bits), stats)
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
    window_ok = any(struct.unpack_from("<H", item, 5)[0] == 0x0A01 for item in window_frames)
    ok = ok and window_ok
    node.get_logger().info(f"自检包窗口 CRC 扫描：{'通过' if window_ok else '失败'}")

    if window_frames:
        test_counter = {
            key: 0
            for key in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")
        }
        test_stats = _fresh_stats()
        test_dedupe = {}
        first = handle_valid_frame(
            window_frames[0], "window", node, test_counter, test_stats,
            test_dedupe, time.time()
        )
        second = handle_valid_frame(
            window_frames[0], "strict", node, test_counter, test_stats,
            test_dedupe, time.time()
        )
        dedupe_ok = first and not second and test_stats["dedup_frames"] == 1
    else:
        dedupe_ok = False
    ok = ok and dedupe_ok
    node.get_logger().info(f"自检 strict/window 去重：{'通过' if dedupe_ok else '失败'}")
    return ok


def connect_grc(node: InfoDecoderNode) -> None:
    try:
        node.grc_rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        if node.apply_camp_to_radio():
            node.get_logger().info(f"XMLRPC 连接正常: {RPC_URL}")
        else:
            node.get_logger().error("XMLRPC 调用失败，请先启动 tx_radio2.py")
    except Exception as exc:
        node.get_logger().error(f"无法连接 XMLRPC({RPC_URL}): {exc}")


def main() -> None:
    rclpy.init()
    node = InfoDecoderNode()

    if SELF_TEST_ON_START and not run_self_test(node):
        node.get_logger().error("f3 离线自检未通过，请勿直接用于赛场！")

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

    node.get_logger().info(
        f"监听 {UDP_IP}:{UDP_PORT} | GRC={GRC_SCRIPT_PATH} | "
        f"profile={ANTIJAM_PROFILE}"
    )

    bit_buffer = ""
    serial_buffer = bytearray()
    packet_window: deque[bytes] = deque(maxlen=PACKET_WINDOW_PAYLOADS)
    dedupe: dict[tuple[int, int, int], float] = {}
    packet_counter = {key: 0 for key in ("0x0A01", "0x0A02", "0x0A03", "0x0A04", "0x0A05")}
    stats = _fresh_stats()
    last_stat = time.time()
    last_udp = time.time()

    try:
        while rclpy.ok():
            now = time.time()
            if node._flush_rx_buffers:
                node._flush_rx_buffers = False
                bit_buffer = ""
                serial_buffer.clear()
                packet_window.clear()
                dedupe.clear()
                node.get_logger().info("阵营切换：已清空 f3 全部接收缓冲")

            if ENABLE_GRC_WATCHDOG and now - last_udp > UDP_WATCHDOG_S:
                node.get_logger().warn("UDP 断流，重启 tx_radio2…")
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
                h = stats["access_hamming"]
                node.get_logger().info(
                    f"[f3 {ANTIJAM_PROFILE} {elapsed:.1f}s] {rates} | "
                    f"AC={stats['ac_hits']} H0/1/2={h.get(0,0)}/{h.get(1,0)}/{h.get(2,0)} "
                    f"反相={stats['ac_hits_inverted']} | "
                    f"Header={stats['header_ok']}/{stats['header_fail']} "
                    f"CRC8/16fail={stats['crc8_fail']}/{stats['crc16_fail']} | "
                    f"strict/window={stats['strict_frames']}/{stats['window_frames']} "
                    f"dedup={stats['dedup_frames']} UDP={stats['udp_bytes']}"
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
