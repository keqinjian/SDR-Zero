#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f5 运行时 RF/FIR 档位表（无 GNU Radio 依赖，解码器与接收入口共用）。"""

from __future__ import annotations

# 只含可热切换字段。判决域 DC / 平滑 / gain_mu 属于启动拓扑，不在此表。
RUNTIME_TUNES = {
    "open": {
        "rf_bandwidth_hz": 1_000_000,
        "rx_gain_db": 40.0,
        "fir_cutoff_hz": 300_000.0,
        "fir_transition_hz": 50_000.0,
    },
    "balanced": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 60_000.0,
    },
    "wide_fir": {
        "rf_bandwidth_hz": 700_000,
        "rx_gain_db": 35.0,
        "fir_cutoff_hz": 280_000.0,
        "fir_transition_hz": 70_000.0,
    },
    "weak_boost": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 40.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 60_000.0,
    },
    "narrow": {
        "rf_bandwidth_hz": 540_000,
        "rx_gain_db": 30.0,
        "fir_cutoff_hz": 255_000.0,
        "fir_transition_hz": 40_000.0,
    },
    "desense": {
        "rf_bandwidth_hz": 600_000,
        "rx_gain_db": 25.0,
        "fir_cutoff_hz": 260_000.0,
        "fir_transition_hz": 20_000.0,
    },
}

# 无有效帧时的探索顺序。
AUTO_TUNE_ORDER = (
    "balanced",
    "wide_fir",
    "weak_boost",
    "narrow",
    "desense",
    "open",
)
