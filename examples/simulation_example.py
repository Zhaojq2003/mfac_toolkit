# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 控制器仿真示例.

本脚本演示如何使用 ``CFDLController`` 与被控对象进行闭环仿真，
并在外部收集时间序列数据。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from mfac_toolkit import DataLogger, MFACConfig, create_controller
from mfac_toolkit.examples.plants import NonlinearDiscretePlant


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
    """运行示例仿真并打印最终结果."""
    dt = 0.01
    config_path = Path(__file__).parent / "config.yaml"
    config = MFACConfig.from_yaml(config_path)
    data = run_simulation(time_steps=400, dt=dt, config=config)

    print(f"=== {config.controller}-MFAC 仿真完成 ===")
    print(f"最终输出 y[-1]: {data['y'][-1]:.4f}")
    print(f"最终控制 u[-1]: {data['u'][-1]:.4f}")
    print(f"最终参考 yd[-1]: {data['yd'][-1]:.4f}")


if __name__ == "__main__":
    main()
