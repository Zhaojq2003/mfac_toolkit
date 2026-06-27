# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 主控制器：编译扩展的薄包装."""

from __future__ import annotations

import contextlib
from collections.abc import Sequence
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from mfac_toolkit.config import MFACConfig
from mfac_toolkit.logger import DataLogger

_core: Any = None
with contextlib.suppress(ImportError):
    import mfac_toolkit._mfac_core as _core


class MFACController(Protocol):
    """MFAC 控制器抽象协议."""

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


def _require_core() -> None:
    if _core is None:
        raise ImportError("MFAC 控制器需要编译后的扩展 _mfac_core")


class CFDLController:
    """基于紧格式动态线性化的无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "CFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.CFDLController(
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=config.initial_phi,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self.y_prev: float = 0.0
        self.u_prev: float = float(config.u0)
        self.u_prev2: float = float(config.u0)
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.y_prev = 0.0
        self.u_prev = float(self.config.u0)
        self.u_prev2 = float(self.config.u0)
        self._step = 0

    def update(self, y: float, yd: float) -> float:
        """计算一个采样步的控制输入."""
        y = float(y)
        yd = float(yd)
        u, delta_y, delta_u, e, phi = self._backend.update(y, yd)
        u_prev_old = self.u_prev
        y_prev_old = self.y_prev
        delta_u_new = u - self.u_prev

        self.u_prev2 = self.u_prev
        self.u_prev = u
        self.y_prev = y

        if self.logger is not None:
            self.logger.log_step(
                step=self._step,
                y=y,
                yd=yd,
                u=u,
                u_prev=u_prev_old,
                y_prev=y_prev_old,
                phi=phi,
                delta_y=delta_y,
                delta_u=delta_u,
                delta_u_new=delta_u_new,
                e=e,
            )
            self._step += 1
        return float(u)

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PPD 估计值（长度为 1 的向量）."""
        return np.array(self._backend.get_phi(), dtype=np.float64)

    def set_phi_hat(self, phi: Sequence[float] | NDArray[np.float64]) -> None:
        """设置当前 PPD 估计值.

        参数:
            phi: 长度为 1 的向量，与 ``get_phi()`` 返回格式一致。
        """
        phi_vec = np.asarray(phi, dtype=np.float64).reshape(-1)
        if phi_vec.shape != (1,):
            raise ValueError(f"CFDL 的 phi_hat 长度应为 1，实际为 {phi_vec.shape}")
        self._backend.set_phi_hat(phi_vec.tolist())


class PFDLController:
    """基于偏格式动态线性化的无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        if config.L_y != 0:
            raise ValueError("PFDL 要求 L_y == 0")
        if config.L_u < 1:
            raise ValueError("PFDL 要求 L_u >= 1")
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "PFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.PFDLController(
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=config.initial_phi,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self._prev_u: float = float(config.u0)
        self._prev_y: float = 0.0
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self._prev_u = float(self.config.u0)
        self._prev_y = 0.0
        self._step = 0

    def update(self, y: float, yd: float) -> float:
        """计算一个采样步的控制输入."""
        y = float(y)
        yd = float(yd)
        u_prev_old = self._prev_u
        y_prev_old = self._prev_y

        if self.logger is not None:
            u, delta_y, delta_u, e, phi = self._backend.update_logged(y, yd)
            delta_u_new = u - u_prev_old
            self._prev_u = u
            self._prev_y = y
            row: dict[str, Any] = {
                "step": self._step,
                "y": y,
                "yd": yd,
                "u": u,
                "u_prev": u_prev_old,
                "y_prev": y_prev_old,
                "delta_y": delta_y,
                "delta_u_new": delta_u_new,
                "e": e,
            }
            for i, value in enumerate(phi):
                row[f"phi_{i}"] = float(value)
            for i, value in enumerate(delta_u):
                row[f"delta_u_{i}"] = float(value)
            self.logger.log_step(**row)
            self._step += 1
        else:
            u = self._backend.update(y, yd)
            self._prev_u = u
            self._prev_y = y
        return float(u)

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PPD 估计向量."""
        return np.array(self._backend.get_phi(), dtype=np.float64)

    def set_phi_hat(self, phi: Sequence[float] | NDArray[np.float64]) -> None:
        """设置当前 PPD 估计向量."""
        phi_vec = np.asarray(phi, dtype=np.float64).reshape(-1)
        if phi_vec.shape != (self.config.L_u,):
            raise ValueError(f"phi_hat 形状应为 ({self.config.L_u},)，实际为 {phi_vec.shape}")
        self._backend.set_phi_hat(phi_vec.tolist())


class FFDLController:
    """基于全格式动态线性化的无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        if config.L_u < 1:
            raise ValueError("FFDL 要求 L_u >= 1")
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "FFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.FFDLController(
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=config.initial_phi,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self._prev_u: float = float(config.u0)
        self._prev_y: float = 0.0
        self._step: int = 0
        self.rho_vector: NDArray[np.float64] | None = None

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self._prev_u = float(self.config.u0)
        self._prev_y = 0.0
        self._step = 0

    def update(self, y: float, yd: float) -> float:
        """计算一个采样步的控制输入."""
        y = float(y)
        yd = float(yd)
        u_prev_old = self._prev_u
        y_prev_old = self._prev_y

        if self.logger is not None:
            u, delta_y, delta_h, e, phi = self._backend.update_logged(y, yd)
            delta_u_new = u - u_prev_old
            self._prev_u = u
            self._prev_y = y
            row: dict[str, Any] = {
                "step": self._step,
                "y": y,
                "yd": yd,
                "u": u,
                "u_prev": u_prev_old,
                "y_prev": y_prev_old,
                "delta_y": delta_y,
                "delta_u_new": delta_u_new,
                "e": e,
            }
            for i, value in enumerate(phi):
                row[f"phi_{i}"] = float(value)
            for i, value in enumerate(delta_h):
                row[f"delta_h_{i}"] = float(value)
            self.logger.log_step(**row)
            self._step += 1
        else:
            u = self._backend.update(y, yd)
            self._prev_u = u
            self._prev_y = y
        return float(u)

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PPD 估计向量."""
        return np.array(self._backend.get_phi(), dtype=np.float64)

    def set_phi_hat(self, phi: Sequence[float] | NDArray[np.float64]) -> None:
        """设置当前 PPD 估计向量."""
        dim = self.config.L_y + self.config.L_u
        phi_vec = np.asarray(phi, dtype=np.float64).reshape(-1)
        if phi_vec.shape != (dim,):
            raise ValueError(f"phi_hat 形状应为 ({dim},)，实际为 {phi_vec.shape}")
        self._backend.set_phi_hat(phi_vec.tolist())

    def set_rho_vector(self, rho: Sequence[float] | NDArray[np.float64]) -> None:
        """设置 FFDL 控制律的 per-component 步长因子."""
        dim = self.config.L_y + self.config.L_u
        rho_vec = np.asarray(rho, dtype=np.float64).reshape(-1)
        if rho_vec.shape != (dim,):
            raise ValueError(f"rho_vector 长度应为 ({dim},)，实际为 {rho_vec.shape}")
        self._backend.set_rho_vector(rho_vec.tolist())
        self.rho_vector = rho_vec


def create_controller(config: MFACConfig, logger: DataLogger | None = None) -> MFACController:
    """根据 ``MFACConfig.controller`` 字段创建对应格式的控制器.

    参数:
        config: 包含 ``controller`` 字段的配置对象。
        logger: 可选的数据记录器；省略时若 ``config.enable_logging`` 为真
            则自动创建默认记录器。

    返回:
        与配置匹配的控制器实例（``CFDLController``、``PFDLController`` 或 ``FFDLController``）。

    异常:
        ValueError: 当 ``controller`` 字段不是 CFDL/PFDL/FFDL 时抛出。
    """
    if config.controller == "CFDL":
        return CFDLController(config, logger=logger)
    if config.controller == "PFDL":
        return PFDLController(config, logger=logger)
    if config.controller == "FFDL":
        return FFDLController(config, logger=logger)
    raise ValueError(f"未知的控制器类型: {config.controller!r}")
