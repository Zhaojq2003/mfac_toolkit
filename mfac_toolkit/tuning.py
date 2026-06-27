# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""直接型 PFDL-MFAC 与间接型 FFDL-MFAC 伪梯度/步长因子整定模块.

本模块为 ``mfac_toolkit._mfac_core`` 中编译扩展整定接口的薄包装，仅负责将
返回结果转换为 NumPy 数组，并保持原有公开 API 不变。
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from mfac_toolkit import _mfac_core as _core
from mfac_toolkit.controller import FFDLController, PFDLController

__all__ = [
    "apply_ffdl_critical_tuning",
    "apply_ffdl_zn_tuning",
    "apply_pfdl_initial_guess",
    "critical_proportional_to_pfdl",
    "ffdl_critical_tuning",
    "ffdl_zn_tuning",
    "pid_to_pfdl",
    "zn_response_to_pfdl",
]


def _as_float_array(values: list[float]) -> NDArray[np.float64]:
    """将编译扩展返回的浮点列表转为 NumPy 数组."""
    return np.array(values, dtype=np.float64)


def pid_to_pfdl(kp: float, ti: float, td: float, ts: float, order: int = 3) -> NDArray[np.float64]:
    """格式 A：由 PID 参数映射到 PFDL-MFAC 伪梯度初值.

    参数:
        kp: 比例增益。
        ti: 积分时间常数，必须为正。
        td: 微分时间常数，必须非负。
        ts: 采样周期，必须为正。
        order: PFDL 伪阶数 ``L_e``，支持 1/2/3。

    返回:
        形状为 ``(order,)`` 的伪梯度初值向量。
    """
    return _as_float_array(_core.pid_to_pfdl(kp, ti, td, ts, order))


def zn_response_to_pfdl(
    k: float,
    tau: float,
    time_delay: float,
    ts: float,
    order: int = 3,
) -> NDArray[np.float64]:
    """格式 B：Z-N 响应曲线法映射到 PFDL-MFAC 伪梯度初值.

    参数:
        k: 对象增益，必须为正。
        tau: 时间常数，必须为正。
        time_delay: 纯时滞，必须为正。
        ts: 采样周期，必须为正。
        order: PFDL 伪阶数 ``L_e``，支持 1/2/3。

    返回:
        形状为 ``(order,)`` 的伪梯度初值向量。
    """
    return _as_float_array(_core.zn_response_to_pfdl(k, tau, time_delay, ts, order))


def critical_proportional_to_pfdl(
    ku: float,
    tu: float,
    ts: float,
    order: int = 3,
) -> NDArray[np.float64]:
    """格式 C：临界比例度法映射到 PFDL-MFAC 伪梯度初值.

    参数:
        ku: 临界增益，必须为正。
        tu: 临界振荡周期，必须为正。
        ts: 采样周期，必须为正。
        order: PFDL 伪阶数 ``L_e``，支持 1/2/3。

    返回:
        形状为 ``(order,)`` 的伪梯度初值向量。
    """
    return _as_float_array(_core.critical_proportional_to_pfdl(ku, tu, ts, order))


def apply_pfdl_initial_guess(
    controller: PFDLController,
    psi: Sequence[float] | NDArray[np.float64],
) -> None:
    """将伪梯度初值注入 ``PFDLController``.

    参数:
        controller: PFDL 控制器实例。
        psi: 伪梯度初值向量，长度必须等于 ``controller.config.L_u``。

    异常:
        ValueError: 当向量长度与控制器伪阶数不匹配时抛出。
    """
    psi_vec = np.asarray(psi, dtype=np.float64).reshape(-1)
    _core.apply_pfdl_initial_guess(controller._backend, psi_vec.tolist())


def ffdl_zn_tuning(
    controller: FFDLController,
    k: float,
    tau: float,
    time_delay: float,
    ts: float,
) -> NDArray[np.float64]:
    """间接型 FFDL 的 Z-N 响应曲线法步长因子整定.

    参数:
        controller: FFDL 控制器实例。
        k: 对象增益，必须为正。
        tau: 时间常数，必须为正。
        time_delay: 纯时滞，必须为正。
        ts: 采样周期，必须为正。

    返回:
        长度 ``L_y + L_u`` 的步长因子向量 ``rho``。
    """
    return _as_float_array(_core.ffdl_zn_tuning(controller._backend, k, tau, time_delay, ts))


def apply_ffdl_zn_tuning(
    controller: FFDLController,
    k: float,
    tau: float,
    time_delay: float,
    ts: float,
) -> None:
    """将 Z-N 响应曲线法整定的步长因子应用到 ``FFDLController``."""
    rho = ffdl_zn_tuning(controller, k, tau, time_delay, ts)
    controller.set_rho_vector(rho)


def ffdl_critical_tuning(
    controller: FFDLController,
    ku: float,
    tu: float,
    ts: float,
) -> NDArray[np.float64]:
    """间接型 FFDL 的临界比例度法步长因子整定.

    参数:
        controller: FFDL 控制器实例。
        ku: 临界增益，必须为正。
        tu: 临界振荡周期，必须为正。
        ts: 采样周期，必须为正。

    返回:
        长度 ``L_y + L_u`` 的步长因子向量 ``rho``。
    """
    return _as_float_array(_core.ffdl_critical_tuning(controller._backend, ku, tu, ts))


def apply_ffdl_critical_tuning(
    controller: FFDLController,
    ku: float,
    tu: float,
    ts: float,
) -> None:
    """将临界比例度法整定的步长因子应用到 ``FFDLController``."""
    rho = ffdl_critical_tuning(controller, ku, tu, ts)
    controller.set_rho_vector(rho)

