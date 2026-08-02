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
RUNTIME_TUNES = {
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

# 无有效帧时的探索顺序：先沿增益阶梯往上爬，再换滤波器形状。
AUTO_TUNE_ORDER = (
    "balanced",
    "weak_boost",
    "high_gain",
    "max_gain",
    "wide_fir",
    "narrow",
    "open",
)
