import socket
import struct
import time
import json
import datetime
import xmlrpc.client # 导入 RPC 库
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ================= 战术配置 =================
MY_CAMP = 'BLUE'    # 修改此处切换阵营: 'RED' 或 'BLUE'
UDP_IP = "127.0.0.1"
UDP_PORT = 14346  
RPC_URL = "http://127.0.0.1:8081"

# 官方信息波频率对照表
INFO_FREQ_MAP = {
    'RED':  433200000,  # 433.2 MHz
    'BLUE': 433920000   # 433.92 MHz
}

# 录制功能配置
RECORD_STREAM = True                
RECORD_PREFIX = "info_record"       

ACCESS_CODE_HEX = "2F6F4C74B914492E"
ACCESS_CODE_BITS = bin(int(ACCESS_CODE_HEX, 16))[2:].zfill(64)
# 【核心防御】：弱信号极易丢失前导，跳过前 16 位，只匹配后 48 位！
FUZZY_CODE = ACCESS_CODE_BITS[16:]

AIR_FRAME_LEN = 27

# ================= 大疆 RM 官方 CRC 校验 =================
CRC8_TAB = [0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35]
CRC16_TAB = [0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78]

def calc_crc8(data):
    crc = 0xFF
    for byte in data: crc = CRC8_TAB[crc ^ byte]
    return crc

def calc_crc16(data):
    crc = 0xFFFF
    for byte in data: crc = ((crc >> 8) ^ CRC16_TAB[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc

def bits_to_bytes(bit_string):
    try:
        return bytes(int(bit_string[i:i+8], 2) for i in range(0, len(bit_string), 8))
    except: return b""

# ================= ROS2 节点 =================
class InfoDecoderNode(Node):
    def __init__(self):
        super().__init__('info_decoder_node')
        self.pub_pos   = self.create_publisher(String, 'radio/info/position', 10)
        self.pub_hp    = self.create_publisher(String, 'radio/info/hp', 10)
        self.pub_ammo  = self.create_publisher(String, 'radio/info/ammo', 10)
        self.pub_macro = self.create_publisher(String, 'radio/info/macro', 10)
        self.pub_buff  = self.create_publisher(String, 'radio/info/buff', 10)
        self.get_logger().info(f'信息波监听已启动[监听阵营: {MY_CAMP}]')

    def publish_data(self, publisher, data_dict):
        msg = String()
        msg.data = json.dumps(data_dict)
        publisher.publish(msg)

# ================= 解析函数 =================
def parse_0x0A01(data, node):
    if len(data) < 24: return
    coords = struct.unpack('<12H', data[:24])
    pos_data = {
        "hero":    {"x": coords[0]/100.0, "y": coords[1]/100.0},
        "eng":     {"x": coords[2]/100.0, "y": coords[3]/100.0},
        "inf3":    {"x": coords[4]/100.0, "y": coords[5]/100.0},
        "inf4":    {"x": coords[6]/100.0, "y": coords[7]/100.0},
        "aerial":  {"x": coords[8]/100.0, "y": coords[9]/100.0},
        "sentry":  {"x": coords[10]/100.0, "y": coords[11]/100.0}
    }
    node.publish_data(node.pub_pos, pos_data)

def parse_0x0A02(data, node):
    if len(data) < 12: return
    hero, eng, inf3, inf4, _, sentry = struct.unpack('<6H', data[:12])
    node.publish_data(node.pub_hp, {"hero": hero, "eng": eng, "inf3": inf3, "inf4": inf4, "sentry": sentry})

def parse_0x0A03(data, node):
    if len(data) < 10: return
    hero, inf3, inf4, aerial, sentry = struct.unpack('<5H', data[:10])
    node.publish_data(node.pub_ammo, {"hero": hero, "inf3": inf3, "inf4": inf4, "aerial": aerial, "sentry": sentry})

def parse_0x0A04(data, node):
    if len(data) < 8: return
    coin, total_coin, status = struct.unpack('<H H I', data[:8])
    macro_data = {"coin": coin, "total_coin": total_coin, "occupation": {"center_highland": (status >> 1) & 0x03, "base_shield": (status >> 8) & 0x01}}
    node.publish_data(node.pub_macro, macro_data)

def parse_0x0A05(data, node):
    if len(data) < 36: return
    buff_data = {"units": {}}
    robot_keys = ["hero", "eng", "inf3", "inf4", "sentry"]
    for i in range(5):
        offset = i * 7
        rec, cool, df, vun, atk = struct.unpack('<B H B B H', data[offset:offset+7])
        buff_data["units"][robot_keys[i]] = {"vulnerability": vun, "attack": atk}
    buff_data["sentry_posture"] = data[35]
    node.publish_data(node.pub_buff, buff_data)

# ================= 核心流程 =================
def main():
    rclpy.init()
    ros_node = InfoDecoderNode()

    # 1. 尝试连接 GNU Radio 遥控器并切频
    try:
        grc_rpc = xmlrpc.client.ServerProxy(RPC_URL)
        target_f = INFO_FREQ_MAP[MY_CAMP]
        grc_rpc.set_target_freq(target_f)
        ros_node.get_logger().info(f"切换到频率: {target_f/1e6} MHz")
    except Exception as e:
        ros_node.get_logger().error(f"无法连接到 GNU Radio XMLRPC (端口 8081): {e}")

    # 2. 初始化黑匣子录制
    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    current_record_file = f"{RECORD_PREFIX}_{MY_CAMP}_{now}.txt"
    file_recorder = None
    if RECORD_STREAM:
        try:
            file_recorder = open(current_record_file, 'w')
            ros_node.get_logger().info(f"录制已开启: {current_record_file}")
        except Exception as e:
            ros_node.get_logger().error(f"录制创建失败: {e}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((UDP_IP, UDP_PORT))
    
    bit_buffer = ""
    serial_buffer = bytearray()
    last_stat_time = time.time()
    packet_counter = {"0x0A01":0, "0x0A02":0, "0x0A03":0, "0x0A04":0, "0x0A05":0}

    try:
        while rclpy.ok():
            current_time = time.time()
            if current_time - last_stat_time > 5.0:
                ros_node.get_logger().info(f"[统计] 5秒捕获: {packet_counter}")
                packet_counter = {k:0 for k in packet_counter}
                last_stat_time = current_time

            try:
                data, _ = sock.recvfrom(16384)
                incoming_bits = ''.join(str(b) for b in data)
                bit_buffer += incoming_bits
                if file_recorder:
                    file_recorder.write(incoming_bits + "\n")
                    file_recorder.flush()
            except BlockingIOError: pass

            # --- 第一层：暴力突防 (无视 Header，只认 Access Code) ---
            while len(bit_buffer) >= AIR_FRAME_LEN * 8:
                idx = bit_buffer.find(FUZZY_CODE)
                if idx == -1: 
                    bit_buffer = bit_buffer[-63:]
                    break
                    
                start_idx = idx - 16
                if start_idx < 0: 
                    bit_buffer = bit_buffer[idx+1:]
                    continue
                    
                if len(bit_buffer) < start_idx + (AIR_FRAME_LEN * 8): 
                    break 

                frame_bits = bit_buffer[start_idx : start_idx + (AIR_FRAME_LEN * 8)]
                frame_bytes = bits_to_bytes(frame_bits)
                
                # ⚡ 暴力逻辑：不再检查 [8:12] 的 Header！
                # 直接切下后面 19 字节 (包含 Header 4字节 + Payload 15字节) 全部塞进大水池
                if frame_bytes:
                    serial_buffer.extend(frame_bytes[12:27])
                    
                bit_buffer = bit_buffer[start_idx + (AIR_FRAME_LEN * 8):]
            
            # --- 第二层：血雨腥风捞 0xA5 (双重 CRC 铁闸) ---
            while len(serial_buffer) >= 9:
                sof_idx = serial_buffer.find(0xA5)
                if sof_idx == -1: 
                    serial_buffer.clear()
                    break
                    
                if sof_idx > 0: 
                    serial_buffer = serial_buffer[sof_idx:]
                    
                if len(serial_buffer) < 5: 
                    break
                
                # 铁闸 1：CRC8
                if calc_crc8(serial_buffer[:4]) != serial_buffer[4]:
                    serial_buffer = serial_buffer[1:]
                    continue
                    
                data_length = struct.unpack('<H', serial_buffer[1:3])[0]
                expected_len = 5 + 2 + data_length + 2
                
                if expected_len > 128: 
                    serial_buffer = serial_buffer[1:]
                    continue
                    
                if len(serial_buffer) < expected_len: 
                    break 
                    
                full_frame = serial_buffer[:expected_len]
                
                # 铁闸 2：CRC16
                if calc_crc16(full_frame[:-2]) == struct.unpack('<H', full_frame[-2:])[0]:
                    # 🚀 恭喜！你在乱码中捞到了真金！
                    cmd_id = struct.unpack('<H', full_frame[5:7])[0]
                    actual_data = full_frame[7 : 7 + data_length]
                    
                    if cmd_id == 0x0A01: parse_0x0A01(actual_data, ros_node); packet_counter["0x0A01"]+=1
                    elif cmd_id == 0x0A02: parse_0x0A02(actual_data, ros_node); packet_counter["0x0A02"]+=1
                    elif cmd_id == 0x0A03: parse_0x0A03(actual_data, ros_node); packet_counter["0x0A03"]+=1
                    elif cmd_id == 0x0A04: parse_0x0A04(actual_data, ros_node); packet_counter["0x0A04"]+=1
                    elif cmd_id == 0x0A05: parse_0x0A05(actual_data, ros_node); packet_counter["0x0A05"]+=1
                    
                    serial_buffer = serial_buffer[expected_len:]
                else:
                    # 漏洞修复：如果 CRC16 失败，只往前挪 1 字节
                    serial_buffer = serial_buffer[1:]
                    
            rclpy.spin_once(ros_node, timeout_sec=0.001)

    except KeyboardInterrupt: pass
    finally:
        if file_recorder: file_recorder.close()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()