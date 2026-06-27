# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""基于仿真数组计算时域性能指标."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def _validate_inputs(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    t: NDArray[np.float64] | np.ndarray | None = None,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """校验并规整指标计算所需的输入.

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        t: 可选时间向量，形状 ``(N,)``。

    返回:
        一维数组 ``y``、``yd`` 与 ``t`` 组成的三元组。

    异常:
        ValueError: 当形状不兼容时抛出。
    """
    y = np.asarray(y, dtype=float).reshape(-1)
    yd = np.asarray(yd, dtype=float).reshape(-1)
    if y.shape != yd.shape:
        raise ValueError("y 与 yd 必须具有相同形状")
    if t is None:
        t = np.arange(len(y), dtype=float)
    else:
        t = np.asarray(t, dtype=float).reshape(-1)
        if t.shape != y.shape:
            raise ValueError("t 必须与 y 具有相同形状")
    return y, yd, t


def _compute_dt(t: NDArray[np.float64]) -> NDArray[np.float64]:
    """返回每个时间样本对应的采样间隔.

    ``dt[i] = t[i+1] - t[i]``（当 i < N-1），最后一个样本重复最后一个
    间隔，因此返回数组与 ``t`` 长度相同。
    """
    diffs = np.diff(t)
    if diffs.size == 0:
        return np.ones_like(t)
    return np.append(diffs, diffs[-1])


def iae(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    t: NDArray[np.float64] | np.ndarray | None = None,
) -> float:
    """计算绝对误差积分（IAE）.

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        t: 可选时间向量。若省略，则假设采样步长为 1。

    返回:
        IAE 标量值。
    """
    y, yd, t = _validate_inputs(y, yd, t)
    error = yd - y
    dt = _compute_dt(t)
    return float(np.sum(np.abs(error) * dt))


def itae(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    t: NDArray[np.float64] | np.ndarray | None = None,
) -> float:
    """计算时间加权绝对误差积分（ITAE）.

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        t: 可选时间向量。若省略，则假设采样步长为 1。

    返回:
        ITAE 标量值。
    """
    y, yd, t = _validate_inputs(y, yd, t)
    error = yd - y
    dt = _compute_dt(t)
    return float(np.sum(t * np.abs(error) * dt))


def ise(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    t: NDArray[np.float64] | np.ndarray | None = None,
) -> float:
    """计算平方误差积分（ISE）.

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        t: 可选时间向量。若省略，则假设采样步长为 1。

    返回:
        ISE 标量值。
    """
    y, yd, t = _validate_inputs(y, yd, t)
    error = yd - y
    dt = _compute_dt(t)
    return float(np.sum(error * error * dt))


def rmse(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
) -> float:
    """计算均方根误差（RMSE）.

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。

    返回:
        RMSE 标量值。
    """
    y, yd, _ = _validate_inputs(y, yd, None)
    error = yd - y
    return float(np.sqrt(np.mean(error * error)))


def overshoot(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
) -> float:
    r"""计算相对于最终参考值的最大超调量.

    指标定义为

    .. math::
        OS = \\max_{k} \\frac{y(k) - y_{d,\\text{final}}}{y_{d,\\text{final}}}

    结果下限截断为 0。若最终参考值为 0，则返回绝对超调量。

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。

    返回:
        最大超调量分数（0.05 表示 5%）。
    """
    y, yd, _ = _validate_inputs(y, yd, None)
    yd_final = float(yd[-1])
    if abs(yd_final) < np.finfo(float).eps:
        return float(np.max(np.abs(y - yd_final)))
    return float(max(0.0, np.max((y - yd_final) / abs(yd_final))))


def settling_time(
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    t: NDArray[np.float64] | np.ndarray | None = None,
    threshold: float = 0.02,
) -> float:
    """计算 2%（可配置）调节时间.

    调节时间定义为：从该时刻起，归一化跟踪误差始终保持在指定误差带内。

    参数:
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        t: 可选时间向量。若省略，则返回索引。
        threshold: 相对于最终参考值的误差带，默认 0.02。

    返回:
        调节时间。若输出始终未进入误差带，则返回最终时刻。
    """
    y, yd, t = _validate_inputs(y, yd, t)
    yd_final = float(yd[-1])

    if abs(yd_final) < np.finfo(float).eps:
        normalized_error = np.abs(y - yd_final)
    else:
        normalized_error = np.abs((y - yd_final) / yd_final)

    within_band = normalized_error <= threshold
    if not np.any(within_band):
        return float(t[-1])

    settled = np.where(within_band)[0]
    for idx in settled:
        if np.all(within_band[idx:]):
            return float(t[idx])
    return float(t[-1])
