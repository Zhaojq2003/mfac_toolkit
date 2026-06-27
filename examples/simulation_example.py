# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 控制器仿真示例.

本脚本完全独立于 ``CFDLController`` 类：它持有被控对象、参考轨迹与数据收集数组。
循环结束后，将这些数组传入 ``mfac_toolkit.analysis`` 与 ``mfac_toolkit.visualization``。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from mfac_toolkit import DataLogger, MFACConfig, create_controller
from mfac_toolkit.analysis import (
    iae,
    ise,
    itae,
    overshoot,
    rmse,
    settling_time,
)
from mfac_toolkit.model import NonlinearDiscretePlant
from mfac_toolkit.visualization import plot_time_response


def generate_reference(time_steps: int, dt: float) -> NDArray[np.float64]:
    """构造分段恒值参考轨迹.

    参数:
        time_steps: 仿真总步数。
        dt: 采样周期。

    返回:
        形状为 ``(time_steps,)`` 的参考数组。
    """
    t = np.arange(time_steps, dtype=float) * dt
    yd = np.zeros_like(t)
    yd[t < 2.0] = 0.0
    yd[(t >= 2.0) & (t < 5.0)] = 1.0
    yd[(t >= 5.0) & (t < 8.0)] = 2.0
    yd[t >= 8.0] = 1.5
    return yd


def run_simulation(
    time_steps: int = 400,
    dt: float = 0.01,
    config: MFACConfig | None = None,
) -> dict[str, NDArray[np.float64]]:
    """运行闭环 MFAC 仿真.

    参数:
        time_steps: 离散时间总步数。
        dt: 采样周期。
        config: 可选的 ``MFACConfig``。省略时使用默认值。

    返回:
        包含时间、参考、输出、控制与 PPD 数组的字典。
    """
    if config is None:
        config_path = Path(__file__).parent / "config.yaml"
        config = MFACConfig.from_yaml(config_path)

    plant = NonlinearDiscretePlant(y0=0.0)

    with DataLogger(enabled=config.enable_logging, log_dir=config.log_dir) as logger:
        controller = create_controller(config, logger=logger)
        logger.set_metadata(
            controller=controller.__class__.__name__,
            controller_format=config.controller,
            plant=plant.__class__.__name__,
            plant_params={"y0": plant.y0},
            time_steps=time_steps,
            dt=dt,
            config=config,
        )

        t = np.arange(time_steps, dtype=float) * dt
        yd = generate_reference(time_steps, dt)
        y = np.zeros(time_steps, dtype=float)
        u = np.zeros(time_steps, dtype=float)
        phi_dim = config.L_y + config.L_u
        phi = np.zeros((time_steps, phi_dim), dtype=float)

        for k in range(time_steps - 1):
            y[k] = plant.y
            u[k] = controller.update(y=y[k], yd=yd[k])
            phi[k, :] = controller.get_phi()
            plant.update(u[k])

        y[-1] = plant.y
        u[-1] = u[-2] if time_steps > 1 else 0.0
        phi[-1, :] = phi[-2, :] if time_steps > 1 else np.full(phi_dim, config.initial_phi)

    return {"t": t, "yd": yd, "y": y, "u": u, "phi": phi}


def main() -> None:
    """运行示例仿真，打印指标并显示曲线."""
    dt = 0.01
    config_path = Path(__file__).parent / "config.yaml"
    config = MFACConfig.from_yaml(config_path)
    data = run_simulation(time_steps=400, dt=dt, config=config)
    t, y, yd, u = data["t"], data["y"], data["yd"], data["u"]

    print(f"=== {config.controller}-MFAC 仿真指标 ===")
    print(f"IAE:   {iae(y, yd, t):.4f}")
    print(f"ITAE:  {itae(y, yd, t):.4f}")
    print(f"ISE:   {ise(y, yd, t):.4f}")
    print(f"RMSE:  {rmse(y, yd):.4f}")
    print(f"超调量:     {overshoot(y, yd):.4f}")
    print(f"调节时间: {settling_time(y, yd, t, threshold=0.02):.4f}")

    out_dir = Path(__file__).parent / "output"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "mfac_time_response.png"

    fig = plot_time_response(t, y, yd, u)
    fig.savefig(out_path, dpi=150)
    print(f"已保存 {out_path}")
    if plt.isinteractive():
        plt.show()


if __name__ == "__main__":
    main()
