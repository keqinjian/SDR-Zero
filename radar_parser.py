import socket
import struct
import binascii

# 1. 网络配置：监听 GNU Radio 发来的 UDP 数据
UDP_IP = "127.0.0.1"
UDP_PORT = 12345

# 2. 比赛协议常量定义
# 官方信息波报头 (红蓝双方相同)
ACCESS_CODE = bytes([0x2F, 0x6F, 0x4C, 0x74, 0xB9, 0x14, 0x49, 0x2E])
# 官方指定的长度头 (15字节 = 0x000F，两个相同的 16bit 小端序)
EXPECTED_HEADER = bytes([0x0F, 0x00, 0x0F, 0x00])
# 帧长：8 + 4 + 15 = 27 字节
FRAME_LEN = 27 

def parse_payload(payload_bytes):
    """
    核心解析函数：把 15 字节的 16 进制数据变成真实的机甲参数。
    由于官方详细协议文档不在图里，这里我为你写一个最典型的 RM 协议解析模板！
    假设这 15 字节的内容是：
    [0]: 机器人ID (1字节 unsigned char)
    [1:5]: X坐标 (4字节 float32)[2:9]: Y坐标 (4字节 float32)
    [9:11]: 剩余血量 (2字节 unsigned short)
    [11:13]: 剩余发弹量 (2字节 unsigned short)[13]: 增益状态 (1字节 unsigned char)
    [14]: 队伍经济 (1字节 unsigned char)
    """
    try:
        # struct.unpack 是 Python 处理底层 C 结构体数据的神器
        # '<' 表示小端序 (Little-Endian，RM 比赛通用)
        # 'B' 表示 1字节整数，'f' 表示 4字节浮点数，'H' 表示 2字节整数
        robot_id, pos_x, pos_y, hp, ammo, buff, economy = struct.unpack('<B f f H H B B', payload_bytes)
        
        print("\n================ ⚡ 成功解析机甲数据 ⚡ ================")
        print(f"🤖 机器人 ID: {robot_id}")
        print(f"📍 位置坐标: X={pos_x:.2f}, Y={pos_y:.2f}")
        print(f"❤️ 剩余血量: {hp}")
        print(f"🔫 剩余弹药: {ammo}")
        print(f"🛡️ 增益状态: {hex(buff)}")
        print(f"💰 队伍经济: {economy}")
        print("========================================================\n")
    except Exception as e:
        print(f"解析 Payload 失败: {e}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"[*] 雷达信号解析系统已启动，正在监听端口 {UDP_PORT} ...")
    
    # 建立一个接收缓冲区，应对网络包的粘包或截断
    buffer = bytearray()
    
    while True:
        # 从 GNU Radio 接收数据 (一次最多收 2048 字节)
        data, addr = sock.recvfrom(2048)
        buffer.extend(data)
        
        # 滑动窗口查找 Access Code
        while len(buffer) >= FRAME_LEN:
            # 在缓冲区中寻找包头
            idx = buffer.find(ACCESS_CODE)
            
            if idx == -1:
                # 没找到包头？说明全是乱码，清空缓冲区（保留最后7个字节防止包头被切断）
                buffer = buffer[-7:]
                break
                
            # 如果找到了包头，检查缓冲区里有没有攒够一整帧的数据 (27字节)
            if len(buffer) < idx + FRAME_LEN:
                break # 数据还不够，等下一个 UDP 包
                
            # 提取出完美的一帧数据！
            frame = buffer[idx : idx + FRAME_LEN]
            header = frame[8:12]
            payload = frame[12:]
            
            # 校验 Header 长度信息是否匹配
            if header == EXPECTED_HEADER:
                # 打印原始十六进制数据（调试用）
                raw_hex = binascii.hexlify(payload).decode('utf-8').upper()
                # print(f"[+] 捕获合法载荷 (Raw Hex): {raw_hex}")
                
                # 调用函数解析出人类可读的信息
                parse_payload(payload)
            else:
                pass # 报头不对，可能是干扰波或误码，直接丢弃
                
            # 处理完毕，把这帧数据从缓冲区里删掉，继续往后找
            buffer = buffer[idx + FRAME_LEN :]

if __name__ == '__main__':
    main()