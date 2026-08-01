#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""f6 运行时档位与 AGC 限幅（无 GNU Radio 依赖）。"""

from __future__ import annotations

AGC_GAIN_MIN_DB = 15.0
AGC_GAIN_MAX_DB = 45.0
AGC_STEP_DB = 2.0

# IQ 削顶 / 过弱阈值（相对 Pluto float IQ 常见幅度，现场可环境变量覆盖）
IQ_CLIP_MAX_ABS = 0.92
IQ_WEAK_MAX_ABS = 0.05
IQ_CLIP_RMS = 0.55

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

AUTO_TUNE_ORDER = (
    "balanced",
    "wide_fir",
    "weak_boost",
    "narrow",
    "desense",
    "open",
)
