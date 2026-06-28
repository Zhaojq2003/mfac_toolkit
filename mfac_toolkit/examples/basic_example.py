# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 最小示例：从 YAML 读取配置并运行闭环仿真.

运行: uv run python -m mfac_toolkit.examples.basic_example

修改 `mfac_toolkit/examples/siso_config.yaml` 即可切换 CFDL/PFDL/FFDL 或调整参数.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from numpy.typing import NDArray

from mfac_toolkit import MFACConfig, create_controller
from mfac_toolkit.examples.plants import NonlinearDiscretePlant


def reference(t: NDArray[np.float64]) -> NDArray[np.float64]:
    """分段恒值参考轨迹."""
    return np.where(t < 2.0, 0.0, np.where(t < 5.0, 1.0, np.where(t < 8.0, 2.0, 1.5)))


def plot_result(
    t: NDArray[np.float64],
    yd: NDArray[np.float64],
    y: NDArray[np.float64],
    u: NDArray[np.float64],
    out_path: Path,
) -> None:
    """保存跟踪曲线与控制输入图."""
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(t, yd, "k--", label="Reference")
    axes[0].plot(t, y, label="Output")
    axes[0].set_ylabel("y")
    axes[0].legend()
    axes[0].grid(True, linestyle=":", alpha=0.6)

    axes[1].step(t, u, where="post")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("u")
    axes[1].grid(True, linestyle=":", alpha=0.6)

    out_path.parent.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)


def main() -> None:
    """运行闭环仿真."""
    dt = 0.01
    n_steps = 400
    t = np.arange(n_steps, dtype=np.float64) * dt
    yd = reference(t)

    config_path = Path(__file__).parent / "siso_config.yaml"
    config = MFACConfig.from_yaml(config_path)
    controller = create_controller(config)

    plant = NonlinearDiscretePlant(y0=0.0)
    y = np.zeros(n_steps)
    u = np.zeros(n_steps)
    for k in range(n_steps - 1):
        y[k] = plant.y
        u[k] = controller.update(y[k], yd[k])
        plant.update(u[k])
    y[-1] = plant.y

    print(f"最终跟踪误差: {abs(yd[-1] - y[-1]):.4f}")
    print(f"最终控制输入: {u[-1]:.4f}")

    out_path = Path(__file__).parent / "output" / "basic_example.png"
    plot_result(t, yd, y, u, out_path)
    print(f"Saved plot to {out_path}")


if __name__ == "__main__":
    main()
