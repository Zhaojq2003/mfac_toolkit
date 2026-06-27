# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 仿真后处理分析工具.

本子包中的所有函数只消费外部传入的 NumPy 数组，不参与实时控制循环。
"""

from __future__ import annotations

from mfac_toolkit.analysis.frequency import pseudo_frequency_response
from mfac_toolkit.analysis.metrics import (
    iae,
    ise,
    itae,
    overshoot,
    rmse,
    settling_time,
)

__all__ = [
    "iae",
    "ise",
    "itae",
    "overshoot",
    "rmse",
    "settling_time",
    "pseudo_frequency_response",
]
