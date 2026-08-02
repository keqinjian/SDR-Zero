#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tx_radio4_tunes.py —— f4 的射频调参面板（不依赖 GNU Radio / ROS）
================================================================

【这个文件是干什么的】
f4 由两个进程组成：tx_radio4.py（收音机）和 info_decoder_f4.py（协议解析）。
两边都要知道"现在用哪个档"，所以这个数字放在这里，两个文件都从这里读，
改一处就够了，不需要设置任何环境变量。

【怎么调（按顺序试）】
1) 先只改下面的 PROFILE，选一个档，直接 `python3 info_decoder_f4.py` 跑 30s，
   看日志里 Header / CRC / 各命令 Hz 有没有变化。
2) 如果想在某个档的基础上单独试增益，改 RX_GAIN_OVERRIDE（不用改档位表）。
3) 都不满意再回到下面 PROFILES 表里改具体字段。

【怎么看频谱图决定增益】
  - 图里原始 IQ（FIR 之前那条）整体压在 -55dBFS 以下、看不到任何顶平
    → 增益不足，信号淹在量化噪声里，应该【加】增益。
  - 图里出现顶平（贴着 0dB 一条直线）
    → 增益过大已经失真，数字处理救不回来，应该【减】增益。
"""

from __future__ import annotations

# =============================================================================
# ★★★★★ 调参面板：现场优先只改这两行 ★★★★★
# =============================================================================

# 用哪个档？可选值见下面 PROFILES 的键：
#   "weak_antijam" 弱信号抗干扰（默认，与 f3 balanced 逐字段对齐）
#   "high_gain"    同上但增益 50dB，频谱整体偏低时先试这个
#   "soft_lock"    旧版激进参数，仅排查用，正常比赛不要用
#   "baseline"     接近 f3 基线，用来确认有没有退步
PROFILE = "weak_antijam"

# 只想单独扫增益时改这里（单位 dB，范围 0~70）。
# 填 None 表示用上面档位里自带的增益。
# 建议扫法：None(35) → 45 → 52 → 58，每档跑 30s 记录 Hz。
RX_GAIN_OVERRIDE = None

# =============================================================================

# 档位表。baseline 与 f3 基线一致；weak_antijam 只改接收机，不动任何空口参数。
#
# 字段含义（小白版）：
#   rf_bandwidth_hz      Pluto 模拟前端带宽，越窄越能挡住邻道干扰
#   rx_gain_db           Pluto 接收增益，弱信号的第一旋钮
#   fir_cutoff_hz        数字低通截止频率，信息波占用约 ±260kHz
#   fir_transition_hz    滤波器过渡带，越窄越陡但阶数越高、群延迟越大
#   fir_window           窗函数，blackman_harris 阻带更深（更抗干扰）
#   complex_dc_length    复数域直流阻塞长度，去掉 Pluto 的本振泄漏
#   post_demod_dc_length 判决域直流阻塞：必须保持 0！长度 256 只有约 5.45 个
#                        码元，会把 Header 里 12 个连续 0 当成直流吃掉
#   smooth_samples       判决前滑动平均长度。注意它【不是】GFSK 的匹配滤波器，
#                        SPS=47 时只会抹平边沿并给时钟环带来群延迟，保持 1
#   gain_mu              M&M 时钟环增益，越大锁得越快但越抖
#   freq_error           频偏预补偿。Pluto 与发射端晶振偏差真实存在，
#                        置 0 等于让环路自己硬拉，弱信号下更难锁
#   omega_relative_limit 码元周期允许的相对漂移范围
PROFILES = {
    "weak_antijam": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
    },
    "high_gain": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 50.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
    },
    "soft_lock": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 40.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
        "fir_window": "blackman_harris",
        "complex_dc_length": 32,
        "post_demod_dc_length": 0,
        "smooth_samples": 5,
        "gain_mu": 0.10,
        "freq_error": 0.0,
        "omega_relative_limit": 0.01,
    },
    "baseline": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 45.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
        "fir_window": "hamming",
        "complex_dc_length": 0,
        "post_demod_dc_length": 0,
        "smooth_samples": 1,
        "gain_mu": 0.175,
        "freq_error": 0.0048,
        "omega_relative_limit": 0.005,
    },
}


def selected_profile() -> dict:
    """取出当前档位的副本，并套用 RX_GAIN_OVERRIDE。"""
    if PROFILE not in PROFILES:
        choices = ", ".join(sorted(PROFILES))
        raise SystemExit(
            f"tx_radio4_tunes.PROFILE={PROFILE!r} 不认识；可选：{choices}"
        )
    profile = dict(PROFILES[PROFILE])
    if RX_GAIN_OVERRIDE is not None:
        profile["rx_gain_db"] = max(0.0, min(70.0, float(RX_GAIN_OVERRIDE)))
    return profile
