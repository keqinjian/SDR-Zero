import xmlrpc.client
import time

# 连接到 GNU Radio 内部的 XMLRPC 服务器
grc_server = xmlrpc.client.ServerProxy("http://127.0.0.1:8080")

print("🔌 成功连接到 GNU Radio 控制中枢！")

# 比赛规则对照表 (频率 Hz, Sensitivity)
jamming_config = {
    1: (432200000, 2.8323), # 红方一级
    2: (432500000, 2.5809), # 红方二级
    3: (432800000, 0.6646), # 红方三级
    4: (434920000, 2.8323), # 蓝方一级
    5: (434620000, 2.5809), # 蓝方二级
    6: (434320000, 0.6646)  # 蓝方三级
}

def switch_jamming_target(level):
    freq, sens = jamming_config.get(level, (None, None))
    if freq is None:
        print("无效的等级")
        return
        
    print(f"🔄 正在指令 GNU Radio 切换到: 难度 {level} (频率: {freq/1e6}MHz, 灵敏度: {sens})")
    
    # 👇 这就是 XMLRPC 的魔法！直接调用 set_变量名()
    grc_server.set_target_freq(freq)
    grc_server.set_target_sens(sens)
    print("✅ 切换完成！")

if __name__ == "__main__":
    # 测试一下动态切频
    print("=== 开始切频测试 ===")
    
    # 切到蓝方三级
    switch_jamming_target(6)
    time.sleep(3) # 让它在这个频率飞一会
    
    # 切回红方二级
    switch_jamming_target(2)
    time.sleep(3)