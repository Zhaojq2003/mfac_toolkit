# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-28

"""MIMO MFAC 最小示例：2×2 线性耦合系统闭环仿真.

运行: uv run python -m mfac_toolkit.examples.mimo_example
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from numpy.typing import NDArray

from mfac_toolkit import MFACConfig, create_controller
from mfac_toolkit.examples.plants import MimoLinearPlant


def plot_result(
    t: NDArray[np.float64],
    yd: NDArray[np.float64],
    y: NDArray[np.float64],
    u: NDArray[np.float64],
    out_path: Path,
) -> None:
    """保存两个通道的跟踪曲线与控制输入图."""
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    dim = y.shape[1]
    for i in range(dim):
        axes[0, i].plot(t, yd[:, i], "k--", label="Reference")
        axes[0, i].plot(t, y[:, i], label="Output")
        axes[0, i].set_ylabel(f"y[{i}]")
        axes[0, i].legend()
        axes[0, i].grid(True, linestyle=":", alpha=0.6)

        axes[1, i].step(t, u[:, i], where="post")
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].set_ylabel(f"u[{i}]")
        axes[1, i].grid(True, linestyle=":", alpha=0.6)

    out_path.parent.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)


def main() -> None:
    """运行 2×2 MIMO 闭环仿真."""
    dt = 0.01
    n_steps = 500
    t = np.arange(n_steps, dtype=np.float64) * dt

    # 分段恒值参考轨迹（两个通道）
    yd = np.zeros((n_steps, 2), dtype=np.float64)
    yd[:, 0] = np.where(t < 1.0, 0.0, np.where(t < 3.0, 1.0, 2.0))
    yd[:, 1] = np.where(t < 2.0, 0.0, np.where(t < 4.0, 1.5, 0.5))

    # 从 YAML 加载 MIMO 配置
    config_path = Path(__file__).parent / "mimo_config.yaml"
    config = MFACConfig.from_yaml(config_path)
    controller = create_controller(config)

    # 2×2 线性被控对象：y = G @ u
    # 使用对角增益，使两个通道独立跟踪，便于观察各通道效果。
    plant = MimoLinearPlant(gain=np.array([[0.5, 0.0], [0.0, 0.6]], dtype=np.float64))
    y = np.zeros((n_steps, 2), dtype=np.float64)
    u = np.zeros((n_steps, 2), dtype=np.float64)

    for k in range(n_steps - 1):
        y[k] = plant.y
        u[k] = controller.update(y[k], yd[k])
        y[k + 1] = plant.update(u[k])

    print(f"最终跟踪误差: {np.abs(yd[-1] - y[-1])}")
    print(f"最终控制输入: {u[-1]}")

    out_path = Path(__file__).parent / "output" / "mimo_example.png"
    plot_result(t, yd, y, u, out_path)
    print(f"Saved plot to {out_path}")


if __name__ == "__main__":
    main()
