#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f5 运行时 RF/FIR 档位表（无 GNU Radio 依赖，解码器与接收入口共用）。

【这个文件是干什么的】
f5 由 tx_radio5.py（收音机）和 info_decoder_f5.py（协议解析）两个进程组成，
两边都要知道"开机用哪个拓扑""可以切哪些档"，所以放在这里统一维护，
改一处两边同时生效，不需要设置任何环境变量。

【怎么调】
1) 一般只改下面的 BOOT_PROFILE。默认 "auto" 会让解码器按 AUTO_TUNE_ORDER
   自动爬增益，绝大多数情况不用管。
2) 想固定住不让它自动切，把 BOOT_PROFILE 改成 "weak_fixed"，
   并把 info_decoder_f5.py 里的 ENABLE_AUTO_TUNE 改成 False。
3) 想改某个档的具体数值，直接改下面 RUNTIME_TUNES 表。

档位设计原则
------------
1. balanced 与 f3 的 balanced 档逐字段对齐（600kHz / 35dB / 260k+20k），
   它是 A/B 基准：f5 若不如 f3，第一嫌疑就是自适应把档切歪了。
2. 其余档位沿“增益从低到高”的单调阶梯排列。实测频谱里原始 IQ 峰值只有
   约 -55dBFS，离 ADC 削顶还有很大余量，因此弱信号时正确方向是**加增益**。
3. desense（低增益）刻意不放进自动探索顺序：f5 没有 IQ 探头，无法证明削顶，
   一旦误切到 25dB 就会把本来就微弱的信息波彻底埋掉。需要时手动指定档位。
"""

from __future__ import annotations

# =============================================================================
# ★★★★★ 调参面板：现场优先只改这一行 ★★★★★
# =============================================================================
# 开机拓扑，可选值见 tx_radio5.py 的 BOOT_PROFILES：
#   "auto"       默认，解码器自动换档
#   "weak_fixed" 固定弱信号档，配合 ENABLE_AUTO_TUNE=False 用来做对照
#   "baseline"   接近 f3 基线，用来确认有没有退步
BOOT_PROFILE = "auto"
# =============================================================================

# 只含可热切换字段。判决域 DC / 平滑 / gain_mu 属于启动拓扑，不在此表。
#
# ---- 关于 fir_cutoff：信息波比想象中宽得多 ----
# 符号率 = 1MHz/47 = 21.28kHz，峰值频偏 = 1.5628*1MHz/2π = 248.7kHz，
# 调制指数 h = 2*248.7/21.28 = 23.4，Carson 带宽 = ±270kHz。
# 沿用已久的 260kHz cutoff 其实比信号本身还窄 10kHz，一直在削自己的边带。
#
# 红方 433.200MHz 接收时，干扰波折算到基带占：一级 -1470~-530kHz、
# 二级 -1130~-270kHz、三级 -525~-275kHz。二/三级和信息波几乎零间隔，
# 滤波器分不开，只能靠 Access/Header/CRC 挡。所以 full_band 用
# 275kHz cutoff + 20kHz 过渡带（阻带 295kHz 起）：收全信息波，同时压住三级。
RUNTIME_TUNES = {
    "full_band": {
        "rf_bandwidth_hz": 700_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 275_000.0,
        "fir_transition_hz": 20_000.0,
    },
    "balanced": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
    "weak_boost": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
    "high_gain": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 52.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
    "max_gain": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 58.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
    "wide_fir": {
        "rf_bandwidth_hz": 700_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 280_000.0,
        "fir_transition_hz": 60_000.0,
    },
    "narrow": {
        "rf_bandwidth_hz": 540_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 255_000.0,
        "fir_transition_hz": 15_000.0,
    },
    "open": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
    },
    # 仅当确认前端削顶（IQ 顶平）时手动使用；不在自动顺序里。
    "desense": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 25.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
}

# 无有效帧时的探索顺序：先保证带宽够（full_band），再沿增益阶梯往上爬，
# 最后才换滤波器形状。带宽不够时加增益没用——被削掉的边带不会因为放大而回来。
AUTO_TUNE_ORDER = (
    "full_band",
    "balanced",
    "weak_boost",
    "high_gain",
    "max_gain",
    "wide_fir",
    "narrow",
    "open",
)
