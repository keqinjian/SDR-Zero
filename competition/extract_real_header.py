INPUT_FILE = "info_record_RED_20260514_095215.txt" # 确认文件名

# 官方密码 0x2F6F...
AC = "0010111101101111010011000111010010111001000101000100100100101110"

def forensic():
    with open(INPUT_FILE, 'r') as f:
        content = f.read().replace('\n', '')
    
    start = 0
    found_count = 0
    print(f"🔍 正在从录像中提取前 5 个包的特征...\n")

    while True:
        idx = content.find(AC, start)
        if idx == -1 or found_count >= 5:
            break
        
        # 提取密码后的 32 位比特 (即 4 字节 Header)
        raw_header_bits = content[idx+64 : idx+64+32]
        # 提取 Header 后的前 16 位比特 (即 Payload 的前 2 字节)
        raw_payload_bits = content[idx+96 : idx+96+16]
        
        # 转成十六进制看真身
        header_hex = hex(int(raw_header_bits, 2))[2:].zfill(8).upper()
        payload_hex = hex(int(raw_payload_bits, 2))[2:].zfill(4).upper()
        
        print(f"第 {found_count+1} 个包:")
        print(f"   - 原始比特位置: {idx}")
        print(f"   - 捕获到的 Header (HEX): {header_hex}")
        print(f"   - Payload 开头 (HEX): {payload_hex}")
        
        start = idx + 1
        found_count += 1

forensic()