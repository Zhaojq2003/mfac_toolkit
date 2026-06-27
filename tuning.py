# Copyright (c) 2026 RobotX. All rights reserved.
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
from mfac_toolkit.config import MFACConfig
from mfac_toolkit.controller import FFDLController, PFDLController

__all__ = [
    "apply_ffdl_critical_tuning",
    "apply_ffdl_zn_tuning",
    "apply_pfdl_initial_guess",
    "critical_proportional_to_pfdl",
    "ffdl_critical_tuning",
    "ffdl_zn_tuning",
    "physical_pfdl_params",
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


def physical_pfdl_params(
    inertia: float,
    dt: float,
    kp: float,
    ki: float,
    kd: float,
    u_max: float,
    order: int = 2,
) -> tuple[NDArray[np.float64], MFACConfig]:
    """基于物理惯量与 PID 参数计算 PFDL-MFAC 推荐初值与配置.

    该函数移植自四旋翼 rate 通道的工程化 MFAC 初始化方法：

    - ``phi1_0 = dt / inertia`` 来自 ``Δω ≈ (dt / J) · Δτ``；
    - ``phi2_0`` 作为阻尼注入项，与 ``Td/dt`` 成正比；
    - ``lambda``、``rho``、``eta``、``mu`` 均按对象尺度与执行器能力缩放。

    参数:
        inertia: 等效转动惯量或惯性系数，必须为正。
        dt: 采样周期，必须为正。
        kp: PID 比例增益，必须为正。
        ki: PID 积分增益。为 0 时忽略积分时间尺度。
        kd: PID 微分增益，必须非负。
        u_max: 控制输入最大幅值（如最大力矩），用于设置饱和与 ``mu``。
        order: PFDL 伪阶数 ``L_u``，至少为 1。

    返回:
        ``(phi0, config)`` 元组：
        - ``phi0``：初始 PPD 向量，长度等于 ``order``；
        - ``config``：推荐的 ``MFACConfig``。

    异常:
        ValueError: 当输入参数非法时抛出。
    """
    if inertia <= 0.0:
        raise ValueError("inertia 必须为正")
    if dt <= 0.0:
        raise ValueError("dt 必须为正")
    if kp <= 0.0:
        raise ValueError("kp 必须为正")
    if kd < 0.0:
        raise ValueError("kd 必须非负")
    if u_max <= 0.0:
        raise ValueError("u_max 必须为正")
    if order < 1:
        raise ValueError("order 至少为 1")

    phi1_0 = dt / inertia

    # 由 PID 参数形成离散时间尺度：Td = Kd/Kp
    td = kd / max(kp, 1e-9)

    # 偏格式 L=2 时，phi2_0 作为阻尼注入项
    kappa_d = min(max(td / max(dt, 1e-9), 0.0), 2.0)
    phi0 = np.zeros(order, dtype=np.float64)
    phi0[0] = phi1_0
    if order >= 2:
        phi0[1] = 0.2 * kappa_d * phi1_0

    # lambda：与控制律分母同量纲，随 Td/dt 略增
    lambda_ = (0.15 + 0.10 * kappa_d) * phi1_0 * phi1_0
    lambda_ = max(lambda_, 1e-4)

    # rho：按 Kp 与惯量尺度确定，并保守限幅到 MFAC 合法范围
    rho_scale = 15.0
    rho = rho_scale * kp * inertia * (lambda_ + phi1_0 * phi1_0)
    rho = min(max(rho, 0.05), 1.0)

    # eta：根据 Kp 大小微调，避免过大抖动或过小收敛慢
    eta = min(max(0.6 + 0.1 * (kp / 5.0), 0.4), 1.2)

    # mu：抑制 Δu_L 小时的数值爆炸，与执行器能力相关
    mu = (0.08 * u_max) ** 2

    # eps：PPD 重置阈值，与 phi1_0 同量级
    eps = max(1e-4, 0.05 * abs(phi1_0))

    config = MFACConfig(
        controller="PFDL",
        L_y=0,
        L_u=order,
        rho=rho,
        lambda_=lambda_,
        eta=eta,
        mu=mu,
        eps=eps,
        initial_phi=phi1_0,
        u0=0.0,
        u_min=-u_max,
        u_max=u_max,
        enable_logging=False,
    )

    return phi0, config
