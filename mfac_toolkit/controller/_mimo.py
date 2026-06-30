# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-28

"""MIMO MFAC 控制器."""

from __future__ import annotations

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


class MimoCfdlController:
    """基于紧格式动态线性化的 MIMO 无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        if config.L_y != 0:
            raise ValueError("MIMO-CFDL 要求 L_y == 0")
        if config.L_u != 1:
            raise ValueError("MIMO-CFDL 要求 L_u == 1")
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "MIMO-CFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.MimoCfdlController(
            dim=config.dim,
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=0.5,
            u0=config.u0,
            m_upper=config.m_upper,
            m_lower=config.m_lower,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
        self._step = 0

    def update(
        self, y: NDArray[np.float64], yd: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """计算一个采样步的控制输入向量."""
        y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
        yd_arr = np.asarray(yd, dtype=np.float64).reshape(-1)
        if y_arr.shape != (self.config.dim,):
            raise ValueError(f"y 形状 {y_arr.shape} ≠ ({self.config.dim},)")
        if yd_arr.shape != (self.config.dim,):
            raise ValueError(f"yd 形状 {yd_arr.shape} ≠ ({self.config.dim},)")

        u_list = self._backend.update(y_arr.tolist(), yd_arr.tolist())
        u = np.array(u_list, dtype=np.float64)

        if self.logger is not None:
            self.logger.log_step(step=self._step, y=y_arr, yd=yd_arr, u=u)
            self._step += 1

        return u

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PJM 矩阵，形状 (dim, dim)."""
        return np.array(self._backend.get_phi()[0], dtype=np.float64)

    def set_phi_hat(self, phi: NDArray[np.float64]) -> None:
        """设置当前 PJM 矩阵."""
        phi_arr = np.asarray(phi, dtype=np.float64)
        if phi_arr.shape != (self.config.dim, self.config.dim):
            raise ValueError(
                f"phi 形状应为 ({self.config.dim}, {self.config.dim})，实际为 {phi_arr.shape}"
            )
        self._backend.set_phi_hat([phi_arr.tolist()])

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、m_upper 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)


class MimoPfdlController:
    """基于偏格式动态线性化的 MIMO 无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        if config.L_y != 0:
            raise ValueError("MIMO-PFDL 要求 L_y == 0")
        if config.L_u < 1:
            raise ValueError("MIMO-PFDL 要求 L_u >= 1")
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "MIMO-PFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.MimoPfdlController(
            dim=config.dim,
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=0.5,
            u0=config.u0,
            m_upper=config.m_upper,
            m_lower=config.m_lower,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
        self._step = 0

    def update(
        self, y: NDArray[np.float64], yd: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """计算一个采样步的控制输入向量."""
        y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
        yd_arr = np.asarray(yd, dtype=np.float64).reshape(-1)
        if y_arr.shape != (self.config.dim,):
            raise ValueError(f"y 形状 {y_arr.shape} ≠ ({self.config.dim},)")
        if yd_arr.shape != (self.config.dim,):
            raise ValueError(f"yd 形状 {yd_arr.shape} ≠ ({self.config.dim},)")

        u_list = self._backend.update(y_arr.tolist(), yd_arr.tolist())
        u = np.array(u_list, dtype=np.float64)

        if self.logger is not None:
            self.logger.log_step(step=self._step, y=y_arr, yd=yd_arr, u=u)
            self._step += 1

        return u

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PJM 矩阵列表，形状 (L_u, dim, dim)."""
        return np.array(self._backend.get_phi(), dtype=np.float64)

    def set_phi_hat(self, phi: NDArray[np.float64]) -> None:
        """设置当前 PJM 矩阵列表."""
        phi_arr = np.asarray(phi, dtype=np.float64)
        expected = (self.config.L_u, self.config.dim, self.config.dim)
        if phi_arr.shape != expected:
            raise ValueError(f"phi 形状应为 {expected}，实际为 {phi_arr.shape}")
        self._backend.set_phi_hat(phi_arr.tolist())

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、m_upper 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)


class MimoFfdlController:
    """基于全格式动态线性化的 MIMO 无模型自适应控制器."""

    def __init__(self, config: MFACConfig, logger: DataLogger | None = None) -> None:
        if config.L_u < 1:
            raise ValueError("MIMO-FFDL 要求 L_u >= 1")
        self.config: MFACConfig = config
        self.logger: DataLogger | None = logger
        if self.logger is None and config.enable_logging:
            self.logger = DataLogger(
                enabled=True,
                log_dir=config.log_dir,
                metadata={
                    "controller": self.__class__.__name__,
                    "controller_format": "MIMO-FFDL",
                    "config": config,
                },
            )
        _require_core()
        self._backend: Any = _core.MimoFfdlController(
            dim=config.dim,
            eta=config.eta,
            mu=config.mu,
            rho=config.rho,
            lambda_=config.lambda_,
            eps=config.eps,
            l_y=config.L_y,
            l_u=config.L_u,
            initial_phi=0.5,
            u0=config.u0,
            m_upper=config.m_upper,
            m_lower=config.m_lower,
        )
        self.set_phi_hat(_broadcast_initial_phi(config))
        self._step: int = 0

    def reset(self) -> None:
        """将控制器重置为初始状态."""
        self._backend.reset()
        self.set_phi_hat(_broadcast_initial_phi(self.config))
        self._step = 0

    def update(
        self, y: NDArray[np.float64], yd: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """计算一个采样步的控制输入向量."""
        y_arr = np.asarray(y, dtype=np.float64).reshape(-1)
        yd_arr = np.asarray(yd, dtype=np.float64).reshape(-1)
        if y_arr.shape != (self.config.dim,):
            raise ValueError(f"y 形状 {y_arr.shape} ≠ ({self.config.dim},)")
        if yd_arr.shape != (self.config.dim,):
            raise ValueError(f"yd 形状 {yd_arr.shape} ≠ ({self.config.dim},)")

        u_list = self._backend.update(y_arr.tolist(), yd_arr.tolist())
        u = np.array(u_list, dtype=np.float64)

        if self.logger is not None:
            self.logger.log_step(step=self._step, y=y_arr, yd=yd_arr, u=u)
            self._step += 1

        return u

    def get_phi(self) -> NDArray[np.float64]:
        """返回当前 PJM 矩阵列表，形状 (L_y + L_u, dim, dim)."""
        return np.array(self._backend.get_phi(), dtype=np.float64)

    def set_phi_hat(self, phi: NDArray[np.float64]) -> None:
        """设置当前 PJM 矩阵列表."""
        phi_arr = np.asarray(phi, dtype=np.float64)
        expected = (self.config.L_y + self.config.L_u, self.config.dim, self.config.dim)
        if phi_arr.shape != expected:
            raise ValueError(f"phi 形状应为 {expected}，实际为 {phi_arr.shape}")
        self._backend.set_phi_hat(phi_arr.tolist())

    def set_params(self, **kwargs: Any) -> None:
        """在线更新标量/边界参数（如 rho、lambda_、m_upper 等）."""
        _set_params(self, **kwargs)

    def reconfigure(self, config: MFACConfig) -> None:
        """使用新配置重建控制器；会重置状态."""
        _reconfigure(self, config)
