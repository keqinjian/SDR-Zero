#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
competition/jamming_mock/mock_tx.py
===================================
干扰波【发射端业务脚本】对齐：通信协议 V2.0.0 + 规则手册 V2.1.0

---------------------------------------------------------------------------
【小白总览】
---------------------------------------------------------------------------

  mock_tx.py（本文件）
       │  组装 0x0A06（6 位 ASCII 密钥，整帧恰好 15 字节）
       │  后面再拼 120 字节随机填充（官方习惯，不是假 cmd 帧）
       │  切成空口包：Access(干扰专用) + Header + 15B
       ▼
  UDP → 127.0.0.1:12347
       ▼
  本目录 tx_radio.py：UDP Source → GFSK Mod → Pluto Sink
       ▼
  另一台 Pluto RX + jamming_decoder_f2.py

---------------------------------------------------------------------------
【等级 / Sens 重要提醒】
---------------------------------------------------------------------------
  换干扰等级时：必须改 tx_radio.py 里的 target_freq + target_sens 后【重启】GRC。
  gfsk_mod 运行时改 Sens 无效；本脚本 XMLRPC 只能帮你改频点。
  一级 Sens=2.8194，二级 2.5681，三级 0.6517（规则 V2 表 5-23）。

---------------------------------------------------------------------------
【本机双板交叉测试约定】
---------------------------------------------------------------------------
  SDR-A 192.168.2.1 = 平时 jamming RX（8080）
  SDR-B 192.168.3.1 = 平时 info    RX（8081）

  测干扰波收发：
    TX = SDR-B（本目录 tx_radio.py，URI 已写 192.168.3.1，XMLRPC 8083，UDP 12347）
    RX = SDR-A（competition/jamming/tx_radio.py + jamming_decoder_f2.py）
    勿同时开 info RX（B 被 TX 占用）
"""

from __future__ import annotations

import argparse
import random
import socket
import struct
import time
import xmlrpc.client

# =============================================================================
# 一、运行配置
# =============================================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 12347
RPC_URL = "http://127.0.0.1:8083"

ROUND_HZ = 10.0           # 官方密钥约 10 Hz
FILLER_BYTES = 120        # 与开源仓库 jamming round 一致：15+120=135
INTER_PACKET_SLEEP_S = 0.001
KEY_ROTATE_ROUNDS = 50    # 未指定 --key 时每 N 轮换一把
APPLY_FREQ_VIA_RPC = True

# =============================================================================
# 二、空口常量
# =============================================================================
#
# 干扰波 Access Code 与信息波不同！0x16E8D377151C712D
# Header 仍是 00 0F 00 0F
#

ACCESS_CODE = bytes.fromhex("16E8D377151C712D")
HEADER = bytes([0x00, 0x0F, 0x00, 0x0F])
AIR_PAYLOAD_LEN = 15
AIR_FRAME_LEN = 27
CMD_JAM_KEY = 0x0A06

KEYS = ["Taurus", "RM2026", "Hacker", "plutos", "Ab12Xy"]

# 己方阵营 + 干扰等级 → (频点 Hz, Sensitivity)
# 规则：干扰源在己方基座，携带对方密钥；RX decoder 也听己方频点。
JAM_TABLE = {
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
    for b in data:
        crc = CRC8_TAB[crc ^ b]
    return crc


def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TAB[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


def build_referee_frame(cmd_id: int, payload: bytes, seq: int) -> bytes:
    header = bytearray(5)
    header[0] = 0xA5
    struct.pack_into("<H", header, 1, len(payload))
    header[3] = seq & 0xFF
    header[4] = calc_crc8(header[:4])
    body = header + struct.pack("<H", cmd_id) + payload
    frame = bytearray(body + b"\x00\x00")
    struct.pack_into("<H", frame, len(frame) - 2, calc_crc16(frame[:-2]))
    return bytes(frame)


def build_jamming_round(key: str, seq: int, fill_seed: int) -> bytes:
    """
    官方干扰一轮：
      - 0x0A06 整帧恰好 15 字节 = 1 个空口 Payload
      - 再跟原始随机字节填充（不是带 CRC 的假裁判帧）
    """
    if len(key) != 6 or not key.isalnum():
        raise ValueError(f"密钥必须是 6 位字母/数字，收到 {key!r}")
    frame = build_referee_frame(CMD_JAM_KEY, key.encode("ascii"), seq)
    if len(frame) != 15:
        raise AssertionError(f"0x0A06 帧长={len(frame)}，应为 15")
    rng = random.Random(fill_seed)
    filler = bytes(rng.randrange(0, 256) for _ in range(FILLER_BYTES))
    return frame + filler


def serial_to_air_packets(serial_stream: bytes) -> list[bytes]:
    pad = (-len(serial_stream)) % AIR_PAYLOAD_LEN
    if pad:
        serial_stream = serial_stream + bytes(pad)
    packets = []
    for i in range(0, len(serial_stream), AIR_PAYLOAD_LEN):
        pkt = ACCESS_CODE + HEADER + serial_stream[i:i + AIR_PAYLOAD_LEN]
        if len(pkt) != AIR_FRAME_LEN:
            raise AssertionError(f"空口包长度异常 {len(pkt)}")
        packets.append(pkt)
    return packets


def self_check() -> None:
    stream = build_jamming_round("Taurus", 1, 0)
    packets = serial_to_air_packets(stream)
    assert len(stream) == 15 + FILLER_BYTES
    assert len(packets) == 9  # 135/15
    assert packets[0][:8] == ACCESS_CODE
    print("[自检] 干扰波组帧 OK：15+120 → 9 空口包，密钥帧 CRC 通过")


def apply_tx_freq(camp: str, level: int) -> None:
    """
    XMLRPC 只改频点。Sens 必须与等级匹配且已在 tx_radio.py 写对并重启。
    level: 1/2/3
    """
    freq, sens = JAM_TABLE[camp][level - 1]
    try:
        rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        rpc.set_target_freq(freq)
        print(f"[RPC] TX 频点 → {camp} 等级{level}: {freq / 1e6:.3f} MHz")
        print(
            f"[注意] 调制 Sens 应为 {sens}；若 tx_radio.py 初值不符，"
            f"请改 target_sens 后重启 GRC（运行时改 Sens 无效）"
        )
    except Exception as exc:
        print(f"[RPC] 切频失败（可忽略，若已在 tx_radio.py 写死）: {exc}")
        print(f"      请手动确认 TX 为 {freq / 1e6:.3f} MHz / Sens={sens}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="干扰波 RF 发射 mock → jamming_mock/tx_radio（一发一收）"
    )
    parser.add_argument("--ip", default=UDP_IP)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--hz", type=float, default=ROUND_HZ)
    parser.add_argument("--key", default=None, help="固定 6 位密钥；默认轮换")
    parser.add_argument(
        "--camp", choices=("RED", "BLUE"), default="RED",
        help="模拟哪方基座干扰源；RX decoder 听同一阵营",
    )
    parser.add_argument(
        "--level", type=int, choices=(1, 2, 3), default=1,
        help="干扰等级 1/2/3（决定频点；Sens 需 GRC 已对齐）",
    )
    parser.add_argument("--no-rpc", action="store_true")
    args = parser.parse_args()

    if args.key is not None and (len(args.key) != 6 or not args.key.isalnum()):
        raise SystemExit("--key 必须是恰好 6 位字母或数字")

    self_check()

    freq, sens = JAM_TABLE[args.camp][args.level - 1]
    if APPLY_FREQ_VIA_RPC and not args.no_rpc:
        apply_tx_freq(args.camp, args.level)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.ip, args.port)
    period = 1.0 / args.hz

    print(
        f"干扰波 RF mock → {args.ip}:{args.port} | {args.hz:.1f} Hz | "
        f"{args.camp} L{args.level} {freq/1e6:.3f}MHz Sens应={sens} | "
        f"filler={FILLER_BYTES}B"
    )
    print("请确认已启动: python3 competition/jamming_mock/tx_radio.py")
    print("RX 听同一阵营同等级频点；TX/RX 用两台不同 Pluto\n")

    key_idx = 0
    seq = 0
    round_n = 0
    try:
        while True:
            t0 = time.time()
            key = args.key if args.key else KEYS[key_idx]
            stream = build_jamming_round(key, seq, fill_seed=round_n)
            seq = (seq + 1) % 256
            packets = serial_to_air_packets(stream)

            for pkt in packets:
                sock.sendto(pkt, target)
                if INTER_PACKET_SLEEP_S > 0:
                    time.sleep(INTER_PACKET_SLEEP_S)

            round_n += 1
            if round_n % 10 == 0:
                print(
                    f"[jam_tx] round={round_n} key={key} "
                    f"air_pkts={len(packets)} serial={len(stream)}B"
                )

            if (
                not args.key
                and KEY_ROTATE_ROUNDS > 0
                and round_n % KEY_ROTATE_ROUNDS == 0
            ):
                key_idx = (key_idx + 1) % len(KEYS)
                print(f"[jam_tx] 轮换密钥 → {KEYS[key_idx]}")

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\n停止干扰波 RF mock")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
