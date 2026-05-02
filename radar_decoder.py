import socket
import struct
import binascii

UDP_IP = "127.0.0.1"
UDP_PORT = 14346

# 大疆 RM 协议固定帧头 SOF
RM_SOF = 0xA5

def parse_0x0A02(data_bytes):
    """
    根据 PDF Page 39-40 解析对方血量信息 (0x0A02)
    总长 12 字节：
    0-1: 1号英雄血量
    2-3: 2号工程血量
    4-5: 3号步兵血量
    6-7: 4号步兵血量
    8-9: 保留位
    10-11: 7号哨兵血量
    """
    try:
        # '<' 表示小端序，6个 'H' 表示 6个无符号短整型(2字节)
        hero, engineer, inf3, inf4, reserved, sentry = struct.unpack('<H H H H H H', data_bytes)
        
        print("\n💥 [机密拦截] 敌方全军血量监控 💥")
        print(f"🔴 敌方英雄 (1): {hero} HP")
        print(f"🚜 敌方工程 (2): {engineer} HP")
        print(f"🔫 敌方步兵 (3): {inf3} HP")
        print(f"🔫 敌方步兵 (4): {inf4} HP")
        print(f"🗼 敌方哨兵 (7): {sentry} HP")
        print("==================================\n")
    except Exception as e:
        print(f"解析 0x0A02 失败: {e}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[*] 战术雷达解析引擎已启动 (端口 {UDP_PORT}) ...")
    
    # 【核心逻辑】：大缓冲区，用于把 15 字节的切片拼接成完整的串口帧
    serial_buffer = bytearray()
    
    while True:
        # 1. 接收从 GNU Radio 传来的 15 字节切片数据
        payload_slice, _ = sock.recvfrom(2048)
        serial_buffer.extend(payload_slice)
        
        # 2. 在缓冲区里寻找完整的大疆串口帧 (开头必须是 0xA5)
        while len(serial_buffer) >= 9: # 一个合法的帧至少要 9 字节 (头5 + ID2 + 尾2)
            
            # 找 SOF (0xA5)
            sof_idx = serial_buffer.find(RM_SOF)
            if sof_idx == -1:
                # 没找到 0xA5，全是垃圾数据，清空
                serial_buffer.clear()
                break
                
            # 把垃圾数据扔掉，让 0xA5 成为开头
            if sof_idx > 0:
                serial_buffer = serial_buffer[sof_idx:]
                
            # 读取 Data Length (偏移量 1 和 2，小端序)
            data_length = struct.unpack('<H', serial_buffer[1:3])[0]
            
            # 计算整个串口帧的预期长度: 头(5) + cmd_id(2) + data_length + 尾(2)
            expected_frame_len = 5 + 2 + data_length + 2
            
            # 检查当前缓冲区收没收够这个长包？(切片拼接等待)
            if len(serial_buffer) < expected_frame_len:
                break # 还没收完，跳出循环，继续去接 UDP 包
                
            # ⚡ 完美！我们拼凑出了一个完整的串口包！
            full_frame = serial_buffer[:expected_frame_len]
            
            # 提取命令码 Cmd_ID (偏移量 5 和 6，小端序)
            cmd_id = struct.unpack('<H', full_frame[5:7])[0]
            
            # 提取纯粹的业务数据
            actual_data = full_frame[7 : 7 + data_length]
            
            # 3. 路由到对应的解析函数
            if cmd_id == 0x0A02:
                parse_0x0A02(actual_data)
            elif cmd_id == 0x0A01:
                print(f"[*] 收到敌方坐标数据，长度 {data_length}")
                # 等你对照 PDF 第 39 页补充
            elif cmd_id == 0x0A06:
                print(f"[!] 警告！收到干扰密钥：{actual_data.decode('ascii', errors='ignore')}")
            else:
                pass # 其他包暂不处理
                
            # 把已经处理完的帧从缓冲区切掉，继续处理后面的
            serial_buffer = serial_buffer[expected_frame_len:]

if __name__ == '__main__':
    main()