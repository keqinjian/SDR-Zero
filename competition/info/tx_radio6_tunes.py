#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f6 运行时档位与 AGC 限幅（无 GNU Radio 依赖）。

【这个文件是干什么的】
f6 由 tx_radio6.py（收音机）和 info_decoder_f6.py（协议解析）两个进程组成，
两边都要知道"开机用哪个拓扑""AGC 允许到多大""软符号走哪个端口"，
所以放在这里统一维护，改一处两边同时生效，不需要设置任何环境变量。

【怎么调】
1) 一般只改下面调参面板里的 BOOT_PROFILE。默认 "auto" 会让解码器根据 IQ 探头
   自动把增益推到合适的工作点，绝大多数情况不用管。
2) 觉得 AGC 不敢加增益 → 调大 AGC_GAIN_MAX_DB 或抬高 IQ_TARGET_RMS_LOW。
3) 觉得 AGC 太容易判成削顶 → 调大 IQ_CLIP_FRACTION。
4) 想彻底固定住不让它动 → BOOT_PROFILE 改 "fixed_balanced"，
   并把 info_decoder_f6.py 里的 ENABLE_AUTO 改成 False。

增益上限为什么放到 60dB
-----------------------
现场频谱截图里，FIR 之前的原始 IQ 峰值只有约 -55dBFS，离满量程还有 50dB 的
空余。也就是说 35~45dB 的档位远远没有用满 ADC 的动态范围，-60dBm 级别的信息
波会掉到量化噪声以下，再高明的解调也救不回来。AD936x 在 433MHz 手动增益可
以给到 70dB 以上，这里留 60dB 上限 + 削顶保护，属于保守但能真正拉开差距。
"""

from __future__ import annotations

# =============================================================================
# ★★★★★ 调参面板：现场优先只改这两行 ★★★★★
# =============================================================================
# 开机拓扑，可选值见 tx_radio6.py 的 BOOT_PROFILES：
#   "auto"           默认，解码器根据 IQ 探头自动 AGC + 换档
#   "fixed_balanced" 固定档，配合 ENABLE_AUTO=False 用来做对照
#   "baseline"       接近 f3 基线，用来确认有没有退步
BOOT_PROFILE = "auto"

# 软符号（float32）走哪个 UDP 端口。硬比特固定 14346，两者不能相同。
SOFT_UDP_PORT = 14347
# =============================================================================

AGC_GAIN_MIN_DB = 15.0
AGC_GAIN_MAX_DB = 60.0
AGC_STEP_DB = 2.0

# ---- 削顶判定（保守：必须“持续到轨”才算）----
# clip_frac 是 |I| 或 |Q| 贴到 0.98 满量程的样本占比。单发脉冲干扰会让 max_abs
# 瞬间冲高，但 clip_frac 仍然极小；只有真的把前端推饱和，占比才会上千分之一。
IQ_CLIP_FRACTION = 1e-3
IQ_CLIP_MAX_ABS = 0.98
IQ_CLIP_RMS = 0.45

# ---- 目标工作点（连续 AGC 往这个区间收敛）----
# 目标是让 IQ 的 RMS 落在满量程的 8%~30%：低于下限说明白白浪费动态范围，
# 高于上限说明离削顶太近，GFSK 的峰均比会先撞墙。
IQ_TARGET_RMS_LOW = 0.08
IQ_TARGET_RMS_HIGH = 0.30
# 低于此 RMS 认为“几乎什么都没收到”，可以放心大步加增益。
IQ_WEAK_RMS = 0.02
IQ_WEAK_MAX_ABS = 0.05

# ---- FIR cutoff 该开多大：按官方参数算出来的，不是拍脑袋 ----
# 符号率      = 1MHz / 47              = 21.28 kHz
# 峰值频偏    = 1.5628 * 1MHz / 2π     = 248.7 kHz
# 调制指数 h  = 2*248.7 / 21.28        = 23.4   ← 远大于 1，是宽带 FSK
# Carson 带宽 = 2*(248.7 + 21.28)      = 540 kHz，即 ±270 kHz
#
# 也就是说信息波真正占到 ±270 kHz。历史上一直用的 260 kHz cutoff 比信号还窄
# 10 kHz，等于长期在削自己的边带；信号强时靠余量还能解，弱信号就先死在这。
#
# 但开宽是有代价的。红方 433.200MHz 接收时，三档干扰波折算到基带是：
#   一级 432.200MHz  占 -1470 ~ -530 kHz   ← 离信息波还有 260 kHz，随便开
#   二级 432.500MHz  占 -1130 ~ -270 kHz   ← 上边沿正好贴着信息波下边沿
#   三级 432.800MHz  占  -525 ~ -275 kHz   ← 只差 5 kHz
# 二/三级干扰和信息波在频谱上基本是零间隔的，没有任何线性滤波器能分开它们；
# 那种场景下只能靠 Access/Header/CRC 三道闸门挡假包。
#
# 所以 full_band 取 275 kHz cutoff + 20 kHz 过渡带（阻带从 295 kHz 起，约 275
# 抽头，和原来 260/20 一个量级，CPU 不会变贵）：既完整收下信息波，又把三级
# 干扰压在阻带里。当前 mock 用的是一级干扰，这档可以放心用。
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
    # 只在 IQ 探头确认持续削顶时才会被选中，不在轮换顺序里。
    "desense": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 25.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
}

# 无帧时的轮换顺序：先让带宽够用（full_band），再沿增益阶梯往上爬，
# 最后才换滤波器形状。带宽不够的时候单纯加增益是没用的——被削掉的边带
# 不会因为放大而回来。
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
