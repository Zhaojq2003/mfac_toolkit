# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 仿真时域响应曲线."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from numpy.typing import NDArray


def plot_time_response(
    t: NDArray[np.float64] | np.ndarray,
    y: NDArray[np.float64] | np.ndarray,
    yd: NDArray[np.float64] | np.ndarray,
    u: NDArray[np.float64] | np.ndarray,
    figsize: tuple[float, float] = (10, 8),
) -> Figure:
    """绘制跟踪响应、跟踪误差与控制输入.

    参数:
        t: 时间向量，形状 ``(N,)``。
        y: 实际输出序列，形状 ``(N,)``。
        yd: 期望输出序列，形状 ``(N,)``。
        u: 控制输入序列，形状 ``(N,)``。
        figsize: 图像尺寸，单位为英寸。

    返回:
        包含三个纵向子图的 ``matplotlib.figure.Figure``。

    异常:
        ValueError: 当输入形状不兼容时抛出。
    """
    t = np.asarray(t, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    yd = np.asarray(yd, dtype=float).reshape(-1)
    u = np.asarray(u, dtype=float).reshape(-1)

    if not (y.shape == yd.shape == u.shape == t.shape):
        raise ValueError("t、y、yd 与 u 必须具有相同形状")

    fig, axes = plt.subplots(3, 1, figsize=figsize, sharex=True)

    axes[0].plot(t, yd, "--", label=r"Reference $y_d$")
    axes[0].plot(t, y, label=r"Output $y$")
    axes[0].set_ylabel("Output")
    axes[0].set_title("MFAC Time Response")
    axes[0].legend(loc="best")
    axes[0].grid(True, linestyle=":", alpha=0.7)

    axes[1].plot(t, yd - y, color="C3", label="Tracking error")
    axes[1].axhline(0.0, color="k", linewidth=0.8)
    axes[1].set_ylabel("Error")
    axes[1].legend(loc="best")
    axes[1].grid(True, linestyle=":", alpha=0.7)

    axes[2].plot(t, u, color="C2", label="Control input")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Control $u$")
    axes[2].legend(loc="best")
    axes[2].grid(True, linestyle=":", alpha=0.7)

    fig.tight_layout()
    return fig
