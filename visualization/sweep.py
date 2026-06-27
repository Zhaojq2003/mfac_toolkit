# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 参数扫描结果可视化."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from numpy.typing import NDArray


def plot_param_sweep(
    param1_values: NDArray[np.float64] | np.ndarray,
    param2_values: NDArray[np.float64] | np.ndarray,
    metric_grid: NDArray[np.float64] | np.ndarray,
    param1_name: str = "Parameter 1",
    param2_name: str = "Parameter 2",
    metric_name: str = "Metric",
    plot_type: str = "heatmap",
    figsize: tuple[float, float] = (8, 6),
) -> Figure:
    """将二维参数扫描结果绘制为热力图或等高线图.

    网格值由外部计算（例如在仿真脚本中扫描 ``rho`` 与 ``lambda_``），
    再传入本函数仅做绘图。

    参数:
        param1_values: 第一个参数轴的网格坐标，形状 ``(M,)``。
        param2_values: 第二个参数轴的网格坐标，形状 ``(N,)``。
        metric_grid: 每个网格点上的性能指标，形状 ``(M, N)``。
        param1_name: 第一个参数标签。
        param2_name: 第二个参数标签。
        metric_name: 指标标签，用于色条或等高线图例。
        plot_type: ``"heatmap"`` 或 ``"contour"``。
        figsize: 图像尺寸，单位为英寸。

    返回:
        包含参数扫描图的 ``matplotlib.figure.Figure``。

    异常:
        ValueError: 当 ``plot_type`` 不支持或网格形状不匹配时抛出。
    """
    p1 = np.asarray(param1_values, dtype=float).reshape(-1)
    p2 = np.asarray(param2_values, dtype=float).reshape(-1)
    grid = np.asarray(metric_grid, dtype=float)

    if grid.shape != (p1.size, p2.size):
        raise ValueError(f"metric_grid 形状 {grid.shape} 与参数网格 ({p1.size}, {p2.size}) 不匹配")
    if plot_type not in {"heatmap", "contour"}:
        raise ValueError("plot_type 必须为 'heatmap' 或 'contour'")

    fig, ax = plt.subplots(figsize=figsize)

    if plot_type == "heatmap":
        im = ax.imshow(
            grid.T,
            aspect="auto",
            origin="lower",
            extent=(p1[0], p1[-1], p2[0], p2[-1]),
            cmap="viridis",
        )
        cbar = fig.colorbar(im, ax=ax)
        cbar.set_label(metric_name)
    else:  # contour
        levels = np.linspace(grid.min(), grid.max(), num=12)
        cs = ax.contour(p1, p2, grid.T, levels=levels, cmap="viridis")
        ax.clabel(cs, inline=True, fontsize=8)
        cbar = fig.colorbar(cs, ax=ax)
        cbar.set_label(metric_name)

    ax.set_xlabel(param1_name)
    ax.set_ylabel(param2_name)
    ax.set_title(f"{metric_name} sweep: {param1_name} vs {param2_name}")

    fig.tight_layout()
    return fig
