# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-28

"""控制器共享协议与辅助函数."""

from __future__ import annotations

import contextlib
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from mfac_toolkit.config import MFACConfig

_core: Any = None
with contextlib.suppress(ImportError):
    import mfac_toolkit._mfac_core as _core


class MFACController(Protocol):
    """SISO MFAC 控制器抽象协议."""

    def update(self, y: float, yd: float) -> float:
        """计算并返回下一时刻控制输入."""
        ...

    def reset(self) -> None:
        """重置控制器状态."""
        ...

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PPD 估计值."""
        ...

    def set_phi_hat(self, phi: Any) -> None:
        """设置 PPD 估计值."""
        ...


class MimoController(Protocol):
    """MIMO MFAC 控制器抽象协议."""

    def update(self, y: NDArray[np.float64], yd: NDArray[np.float64]) -> NDArray[np.float64]:
        """计算并返回下一时刻控制输入向量."""
        ...

    def reset(self) -> None:
        """重置控制器状态."""
        ...

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PJM 估计矩阵（或矩阵列表）."""
        ...

    def set_phi_hat(self, phi: NDArray[np.float64]) -> None:
        """设置 PJM 估计矩阵."""
        ...


def _require_core() -> None:
    if _core is None:
        raise ImportError("MFAC 控制器需要编译后的扩展 _mfac_core")


def _expected_phi_shape(config: MFACConfig) -> tuple[int, ...]:
    """返回 config 对应的控制器格式所要求的 phi/PJM 形状."""
    if config.dim == 1:
        if config.controller == "CFDL":
            return (1,)
        if config.controller == "PFDL":
            return (config.L_u,)
        # FFDL
        return (config.L_y + config.L_u,)

    if config.controller == "CFDL":
        return (config.dim, config.dim)
    if config.controller == "PFDL":
        return (config.L_u, config.dim, config.dim)
    # FFDL
    return (config.L_y + config.L_u, config.dim, config.dim)


def _broadcast_initial_phi(config: MFACConfig) -> NDArray[np.float64]:
    """将 config.initial_phi 广播为当前控制器格式要求的形状.

    标量会广播为 SISO 重复向量或 MIMO 对角矩阵；数组会直接校验形状。
    """
    expected = _expected_phi_shape(config)
    phi = np.asarray(config.initial_phi, dtype=np.float64)

    if phi.ndim == 0:
        value = phi.item()
        if config.dim == 1:
            return np.full(expected, value, dtype=np.float64)
        base = np.eye(config.dim, dtype=np.float64) * value
        if config.controller == "CFDL":
            return base
        return np.tile(base, (expected[0], 1, 1))

    if phi.shape != expected:
        raise ValueError(
            f"initial_phi 形状应为 {expected}，实际为 {phi.shape} "
            f"(controller={config.controller}, dim={config.dim}, "
            f"L_y={config.L_y}, L_u={config.L_u})"
        )
    return phi
