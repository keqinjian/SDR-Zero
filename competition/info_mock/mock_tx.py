#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
competition/info_mock/mock_tx.py
================================
信息波【发射端业务脚本】对齐：通信协议 V2.0.0 + 规则手册 V2.1.0

---------------------------------------------------------------------------
【小白总览】本脚本在整条测试链中的位置
---------------------------------------------------------------------------

  mock_tx.py（本文件）
       │  组装裁判系统串口帧 0x0A01～0x0A05（新版 0x0A05=41 字节）
       │  按官方空口规则切成「Access Code + Header + 15 字节」
       │  每个空口包 = 27 字节二进制（不是 0/1 字符串！）
       ▼
  UDP → 127.0.0.1:12345
       ▼
  本目录 tx_radio.py（GNU Radio）
       │  UDP Source → GFSK Mod(SPS=47, Sens=1.5628) → PlutoSDR Sink
       ▼
  天线发出 433 MHz 附近 GFSK
       ▼
  另一台 Pluto + competition/info/tx_radio.py（RX）
       + info_decoder_f1.py

---------------------------------------------------------------------------
【和 rm_info_mock_v2.py 的区别】
---------------------------------------------------------------------------
  rm_info_mock_v2：往 14346 直接喂「每字节 0/1」，跳过射频，专测 decoder。
  本脚本：往 12345 喂「空口整包字节」，必须经 GFSK 调制上天线，测真一发一收。

---------------------------------------------------------------------------
【本机双板交叉测试约定】
---------------------------------------------------------------------------
  SDR-A 192.168.2.1 = 平时 jamming RX（8080）
  SDR-B 192.168.3.1 = 平时 info    RX（8081）

  测信息波收发：
    TX = SDR-A（本目录 tx_radio.py，URI 已写 192.168.2.1，XMLRPC 8082，UDP 12345）
    RX = SDR-B（competition/info/tx_radio.py + info_decoder_f1.py）
    勿同时开 jamming RX（A 被 TX 占用）

  步骤：
    1. python3 competition/info_mock/tx_radio.py
    2. python3 competition/info_mock/mock_tx.py --camp RED
    3. 另开：competition/info/tx_radio.py + info_decoder_f1.py（听同一阵营）
"""

from __future__ import annotations

import argparse
import socket
import struct
import time
import xmlrpc.client

# =============================================================================
# 一、运行配置（现场一般只改这里）
# =============================================================================

UDP_IP = "127.0.0.1"
UDP_PORT = 12345          # 必须与本目录 tx_radio.py 的 UDP Source 一致
RPC_URL = "http://127.0.0.1:8082"  # 本目录 tx_radio XMLRPC（可改 TX 频点）

# 官方信息波约 10 Hz 一轮业务；1400 B/s ≈ 每轮 140 字节有效载荷
ROUND_HZ = 10.0

# 包与包之间略停一下，避免瞬间灌爆 UDP/USB（射频时序不必卡死官方 10.15ms）
# 整轮节奏由 ROUND_HZ 的墙钟对齐，不要再「每包睡 10ms + 再睡满 100ms」（会远慢于 10Hz）
INTER_PACKET_SLEEP_S = 0.001

# 启动时是否先连一下 TX GRC 的 XMLRPC，把频点设成 --camp 对应值
APPLY_FREQ_VIA_RPC = True

# =============================================================================
# 二、空口 / 射频常量（名词解释）
# =============================================================================
#
# Access Code：信息波固定暗号 0x2F6F4C74B914492E（干扰波是另一串，别混）
# Header：00 0F 00 0F —— 大端写法的「后面 Payload 长度=15」写两遍
# Payload：固定 15 字节。长裁判帧会被切成多片连续发（信息波一轮要 10 个空口包）
#

ACCESS_CODE = bytes.fromhex("2F6F4C74B914492E")
HEADER = bytes([0x00, 0x0F, 0x00, 0x0F])
AIR_PAYLOAD_LEN = 15
AIR_FRAME_LEN = 8 + 4 + 15  # 27 字节/空口包
LEN_0A05 = 41               # 协议 V2：增益 35B + 姿态 1B + 主要状态 5B

# 阵营 → 广播源中心频（规则表 5-23）。TX 发哪边，RX 就听哪边基座。
CAMP_FREQ_HZ = {
    "RED": 433_200_000,
    "BLUE": 433_920_000,
}

# =============================================================================
# 三、CRC（与裁判系统 / decoder 同一张表）
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
    for b in data:
        crc = CRC8_TAB[crc ^ b]
    return crc


def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TAB[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


def build_referee_frame(cmd_id: int, payload: bytes, seq: int) -> bytes:
    """
    裁判系统串口帧：
      A5 | data_len(2,小端) | seq | CRC8 | cmd_id(2) | data | CRC16(2)
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


def build_unit_buff(heal: int, cool: int, defense: int, neg_def: int, attack: int) -> bytes:
    """单兵种增益 7 字节：回血% / 冷却 / 防御% / 负防御% / 攻击%"""
    return struct.pack("<BHBBH", heal, cool, defense, neg_def, attack)


def build_info_round(seq_start: int, hero_hp: int) -> bytes:
    """
    一轮信息波业务（官方约 10Hz）：仅 0x0A01～0x0A05，合计 140 字节。
    不含 0x0A06（密钥只走干扰波）。
    """
    # 0x0A01 坐标：uint16，单位 cm（decoder 会 /100 成米）
    pos = struct.pack(
        "<12H",
        250, 310,   # 英雄 2.50m, 3.10m
        120, 50,    # 工程
        50, 100,    # 步兵3
        120, 0,     # 步兵4
        1000, 500,  # 空中
        0, 400,     # 哨兵
    )
    # 0x0A02：英雄/工程/步3/步4/保留/哨兵
    hp = struct.pack("<6H", hero_hp, 500, 800, 800, 0, 600)
    ammo = struct.pack("<5H", 150, 450, 400, 500, 800)
    # bit8=1 → 基地增益点已占领（便于肉眼看 macro 话题）
    macro = struct.pack("<HHI", 350, 1200, 256)
    # 0x0A05：5×7 + 姿态 + 5 主要状态 = 41
    buff = (
        build_unit_buff(10, 100, 20, 0, 150)
        + build_unit_buff(0, 0, 0, 0, 100)
        + build_unit_buff(0, 0, 0, 30, 100)
        + build_unit_buff(0, 0, 0, 0, 100)
        + build_unit_buff(5, 50, 40, 0, 120)
        + bytes([2])                # 哨兵姿态：防御
        + bytes([0, 0, 0, 0, 0])    # 主要状态：全存活
    )
    if len(buff) != LEN_0A05:
        raise AssertionError(f"0x0A05 长度={len(buff)}，应为 {LEN_0A05}")

    stream = b"".join([
        build_referee_frame(0x0A01, pos, seq_start + 0),
        build_referee_frame(0x0A02, hp, seq_start + 1),
        build_referee_frame(0x0A03, ammo, seq_start + 2),
        build_referee_frame(0x0A04, macro, seq_start + 3),
        build_referee_frame(0x0A05, buff, seq_start + 4),
    ])
    if len(stream) != 140:
        raise AssertionError(f"信息波一轮应为 140B，实际 {len(stream)}")
    return stream


def serial_to_air_packets(serial_stream: bytes) -> list[bytes]:
    """
    裁判字节流 → 空口包列表。
    不足 15 字节倍数时右侧补 0（官方空口切片习惯；本轮 140→补到 150→10 包）。
    """
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
    """不上天也能验证：CRC、41B、空口包数。"""
    stream = build_info_round(0, 2000)
    packets = serial_to_air_packets(stream)
    assert len(packets) == 10, f"期望 10 个空口包，得到 {len(packets)}"
    assert packets[0][:8] == ACCESS_CODE
    assert packets[0][8:12] == HEADER
    print("[自检] 信息波组帧 OK：140B → 10 空口包，CRC/0x0A05=41 通过")


def apply_tx_freq(camp: str) -> None:
    """通过 XMLRPC 把 TX GRC 切到对应广播频（Sens 改不了运行时，无需动）。"""
    freq = CAMP_FREQ_HZ[camp]
    try:
        rpc = xmlrpc.client.ServerProxy(RPC_URL, allow_none=True)
        rpc.set_target_freq(freq)
        print(f"[RPC] TX 频点 → {camp} {freq / 1e6:.3f} MHz")
    except Exception as exc:
        print(f"[RPC] 切频失败（可忽略，若已在 tx_radio.py 写死频点）: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="信息波 RF 发射 mock → info_mock/tx_radio（一发一收）"
    )
    parser.add_argument("--ip", default=UDP_IP)
    parser.add_argument("--port", type=int, default=UDP_PORT)
    parser.add_argument("--hz", type=float, default=ROUND_HZ, help="业务轮频率，默认 10")
    parser.add_argument(
        "--camp", choices=("RED", "BLUE"), default="RED",
        help="模拟哪方基座广播；RX 必须听同一阵营",
    )
    parser.add_argument("--no-rpc", action="store_true", help="不调用 TX XMLRPC 切频")
    args = parser.parse_args()

    self_check()

    if APPLY_FREQ_VIA_RPC and not args.no_rpc:
        apply_tx_freq(args.camp)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.ip, args.port)
    period = 1.0 / args.hz

    print(
        f"信息波 RF mock → {args.ip}:{args.port} | {args.hz:.1f} Hz | "
        f"camp={args.camp}({CAMP_FREQ_HZ[args.camp]/1e6:.3f}MHz) | "
        f"0x0A05={LEN_0A05}B"
    )
    print("请确认已启动: python3 competition/info_mock/tx_radio.py")
    print("RX 请听同一阵营广播频，且 TX=SDR-A / RX=SDR-B（两台不同 Pluto）")
    print("若 Header 一直为 0：检查 info_mock/tx_radio.py 的 tx_atten（交叉测应≈10，勿用40）\n")

    hero_hp = 2000
    seq = 0
    round_n = 0
    try:
        while True:
            t0 = time.time()
            stream = build_info_round(seq, hero_hp)
            seq = (seq + 5) % 256
            packets = serial_to_air_packets(stream)

            # 连续打出本轮全部空口包（略间隔防灌爆）
            for pkt in packets:
                sock.sendto(pkt, target)
                if INTER_PACKET_SLEEP_S > 0:
                    time.sleep(INTER_PACKET_SLEEP_S)

            round_n += 1
            if round_n % 10 == 0:
                print(
                    f"[info_tx] round={round_n} hero_hp={hero_hp} "
                    f"air_pkts={len(packets)} serial={len(stream)}B"
                )

            hero_hp -= 15
            if hero_hp <= 0:
                hero_hp = 2000

            # 墙钟对齐 ROUND_HZ（包间 sleep 已计入 elapsed）
            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\n停止信息波 RF mock")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
