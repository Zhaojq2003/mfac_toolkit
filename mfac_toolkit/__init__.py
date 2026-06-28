# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC：无模型自适应控制工具包.

本包提供基于紧格式（CFDL）、偏格式（PFDL）与全格式（FFDL）动态
线性化的无模型自适应控制器，适用于 SISO/MIMO 离散时间系统。

控制循环本身不记录历史数据：用户需在外部仿真脚本中收集时间序列，
再使用自定义或第三方工具进行分析与可视化。
"""

from __future__ import annotations

from mfac_toolkit.config import MFACConfig
from mfac_toolkit.controller import (
    CFDLController,
    FFDLController,
    MimoCfdlController,
    MimoFfdlController,
    MimoPfdlController,
    PFDLController,
    create_controller,
)
from mfac_toolkit.logger import DataLogger
from mfac_toolkit.tuning import (
    apply_ffdl_critical_tuning,
    apply_ffdl_zn_tuning,
    apply_pfdl_initial_guess,
    critical_proportional_to_pfdl,
    ffdl_critical_tuning,
    ffdl_zn_tuning,
    pid_to_pfdl,
    zn_response_to_pfdl,
)

__all__ = [
    "CFDLController",
    "PFDLController",
    "FFDLController",
    "MimoCfdlController",
    "MimoPfdlController",
    "MimoFfdlController",
    "create_controller",
    "MFACConfig",
    "DataLogger",
    "apply_ffdl_critical_tuning",
    "apply_ffdl_zn_tuning",
    "apply_pfdl_initial_guess",
    "critical_proportional_to_pfdl",
    "ffdl_critical_tuning",
    "ffdl_zn_tuning",
    "pid_to_pfdl",
    "zn_response_to_pfdl",
]
