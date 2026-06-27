# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""PFDL-MFAC 伪梯度初值映射仿真示例.

本脚本以格式 A（PID 参数 → PFDL-MFAC）为例，演示如何：

1. 由已知 PID 参数计算 Le=3 的伪梯度初值 ``psi_ini``；
2. 将该初值注入 ``PFDLController``；
3. 与被控对象闭环仿真并对比默认初值的跟踪效果。

运行::

    uv run python examples/tuning_example.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from numpy.typing import NDArray

from mfac_toolkit import MFACConfig, PFDLController
from mfac_toolkit.examples.plants import StateSpacePlant
from mfac_toolkit.tuning import apply_pfdl_initial_guess, pid_to_pfdl


def run_closed_loop(
    controller: PFDLController,
    plant: StateSpacePlant,
    yd: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """运行一次闭环仿真.

    参数:
        controller: 已配置好初值的 PFDL 控制器。
        plant: 离散时间被控对象。
        yd: 期望输出序列。

    返回:
        被控对象输出序列 ``y`` 与控制输入序列 ``u``。
    """
    n = len(yd)
    y = np.zeros(n, dtype=np.float64)
    u = np.zeros(n, dtype=np.float64)

    controller.reset()
    plant.reset()

    for k in range(n):
        u[k] = controller.update(y[k], yd[k])
        if k + 1 < n:
            y[k + 1] = plant.update(u[k])

    return y, u


def main() -> None:
    """主仿真流程."""
    ts = 0.05
    n_steps = 200

    # 一阶离散被控对象：y(k+1) = 0.95 y(k) + 0.1 u(k)
    plant = StateSpacePlant(
        A=np.array([[0.95]]),
        B=np.array([[0.1]]),
        C=np.array([[1.0]]),
    )

    # 阶跃参考信号
    yd = np.ones(n_steps, dtype=np.float64)

    # 基础 PFDL-MFAC 配置（Le=3）
    cfg = MFACConfig(
        L_y=0,
        L_u=3,
        rho=0.6,
        lambda_=0.1,
        eta=0.5,
        mu=1.0,
        u0=0.0,
    )

    # 1) 使用默认初值 phi=0.5
    ctrl_default = PFDLController(cfg)
    y_default, u_default = run_closed_loop(ctrl_default, plant, yd)

    # 2) 使用格式 A 映射得到的伪梯度初值
    psi_ini = pid_to_pfdl(kp=0.5, ti=0.5, td=0.05, ts=ts)
    ctrl_tuned = PFDLController(cfg)
    apply_pfdl_initial_guess(ctrl_tuned, psi_ini)
    y_tuned, u_tuned = run_closed_loop(ctrl_tuned, plant, yd)

    # 绘图并保存
    t = np.arange(n_steps, dtype=np.float64) * ts
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    axes[0].plot(t, yd, "k--", label=r"Reference $y_d$")
    axes[0].plot(t, y_default, "b-", label=r"Default initial $\phi=0.5$")
    axes[0].plot(t, y_tuned, "r-", label=r"Tuned initial $\psi_{\mathrm{ini}}$")
    axes[0].set_ylabel("Output")
    axes[0].legend()
    axes[0].grid(True, linestyle=":", alpha=0.6)

    axes[1].step(t, u_default, "b-", where="post", label="Default")
    axes[1].step(t, u_tuned, "r-", where="post", label="Tuned")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Control input")
    axes[1].legend()
    axes[1].grid(True, linestyle=":", alpha=0.6)

    out_dir = Path(__file__).parent / "output"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "tuning_example.png"

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Saved comparison plot to {out_path}")
    print(f"Tuned initial psi = {psi_ini}")


if __name__ == "__main__":
    main()
