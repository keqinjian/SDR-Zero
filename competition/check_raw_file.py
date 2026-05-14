
INPUT_FILE = "info_record_RED_20260514_095215.txt" # 修改为你刚才那个文件名

def check():
    with open(INPUT_FILE, 'r') as f:
        content = f.read().replace('\n', '')
    
    print(f"📊 文件总比特数: {len(content)}")
    if len(content) == 0:
        print("❌ 文件是空的！检查 GNU Radio 是否真的有数据传出。")
        return

    # 1. 检查是不是全是 1 或 0 (被 Squelch 拦死)
    ones = content.count('1')
    zeros = content.count('0')
    print(f"📈 1 的比例: {ones/len(content)*100:.1f}%, 0 的比例: {zeros/len(content)*100:.1f}%")

    # 2. 暴力搜索 Access Code 的所有变体
    AC = "0010111101101111010011000111010010111001000101000100100100101110" # 0x2F6F...
    AC_INV = "".join(['1' if b == '0' else '0' for b in AC]) # 极性翻转版

    if AC in content:
        print("✅ 发现原始密码！说明匹配逻辑没问题，可能是 Header 对不上。")
    elif AC_INV in content:
        print("🚨 发现【极性翻转】版密码！说明你 GRC 里的极性反了，0 变成了 1。")
    else:
        print("❌ 没发现任何密码变体。")
        print(f"👀 前 128 位采样: {content[:128]}")

check()
