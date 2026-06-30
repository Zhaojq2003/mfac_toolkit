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
from mfac_toolkit.logger import DataLogger

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


_STRUCTURAL_FIELDS: frozenset[str] = frozenset({"controller", "dim", "L_y", "L_u"})


def _validate_param_update(config: MFACConfig, **kwargs: Any) -> MFACConfig:
    """校验在线参数更新，返回新的已验证配置.

    拒绝结构性字段；标量/边界字段通过构造新 ``MFACConfig`` 完成范围校验。
    """
    overlap = _STRUCTURAL_FIELDS & kwargs.keys()
    if overlap:
        names = ", ".join(sorted(overlap))
        raise ValueError(
            f"结构性参数（{names}）不能通过 set_params 在线调整，请使用 reconfigure(config)"
        )
    merged = {**config.model_dump(), **kwargs}
    return MFACConfig(**merged)


def _update_logger_config(logger: DataLogger | None, config: MFACConfig) -> None:
    """同步更新记录器元数据中的配置对象."""
    if logger is not None:
        logger.set_metadata(config=config)


def _apply_params(backend: Any, config: MFACConfig, **kwargs: Any) -> None:
    """将已校验的参数变更应用到 Rust 后端."""
    for key in kwargs:
        if key == "lambda_":
            backend.set_lambda(config.lambda_)
        elif key == "initial_phi":
            if config.dim == 1:
                phi = _broadcast_initial_phi(config)
                if config.controller == "CFDL":
                    backend.set_initial_phi(phi.item())
                else:
                    backend.set_initial_phi(phi.tolist())
            else:
                backend.set_initial_phi(float(np.asarray(config.initial_phi, dtype=np.float64).item()))
        else:
            getattr(backend, f"set_{key}")(getattr(config, key))
