# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-28

"""SISO MFAC 控制器."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray

from mfac_toolkit.config import MFACConfig
from mfac_toolkit.controller._base import (
    _apply_params,
    _broadcast_initial_phi,
    _core,
    _require_core,
    _update_logger_config,
    _validate_param_update,
)
from mfac_toolkit.logger import DataLogger


def _set_params(ctrl: Any, **kwargs: Any) -> None:
    """在线更新标量/边界参数，已通过 ``MFACConfig`` 校验."""
    new_config = _validate_param_update(ctrl.config, **kwargs)
    _apply_params(ctrl._backend, new_config, **kwargs)
    ctrl.config = new_config
    _update_logger_config(ctrl.logger, new_config)


def _reconfigure(ctrl: Any, config: MFACConfig) -> None:
    """用新配置重建后端；拒绝更改控制器格式或 SISO/MIMO 类别."""
    if config.controller != ctrl.config.controller:
        raise ValueError(
            f"reconfigure 不允许更改控制器格式："
            f"{ctrl.config.controller} -> {config.controller}"
        )
    if (config.dim == 1) != (ctrl.config.dim == 1):
        raise ValueError("reconfigure 不允许在 SISO (dim=1) 与 MIMO (dim>=2) 之间切换")
    new_controller = type(ctrl)(config, logger=ctrl.logger)
    ctrl._backend = new_controller._backend
    ctrl.config = config
    ctrl.reset()
    ctrl._step = 0
    _update_logger_config(ctrl.logger, config)


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
            initial_phi=0.5,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self.y_prev: float = 0.0
        self.u_prev: float = float(config.u0)
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
        self.y_prev = 0.0
        self.u_prev = float(self.config.u0)
        self._step = 0

    def update(self, y: float, yd: float) -> float:
        """计算一个采样步的控制输入."""
        y = float(y)
        yd = float(yd)
        u, delta_y, delta_u, e, phi = self._backend.update(y, yd)
        u_prev_old = self.u_prev
        y_prev_old = self.y_prev
        delta_u_new = u - self.u_prev

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

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、u_min 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)


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
            initial_phi=0.5,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self._prev_u: float = float(config.u0)
        self._prev_y: float = 0.0
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
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

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、u_min 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)


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
            initial_phi=0.5,
            u0=config.u0,
            u_min=config.u_min,
            u_max=config.u_max,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self._prev_u: float = float(config.u0)
        self._prev_y: float = 0.0
        self._step: int = 0
        self.rho_vector: NDArray[np.float64] | None = None

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
        self._prev_u = float(self.config.u0)
        self._prev_y = 0.0
        self._step = 0
        self.rho_vector = None

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

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、u_min 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)
