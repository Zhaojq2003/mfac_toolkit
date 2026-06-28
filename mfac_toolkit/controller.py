# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 主控制器：编译扩展的薄包装."""

from __future__ import annotations

import contextlib
from collections.abc import Callable, Sequence
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


_ControllerCtor = Callable[[MFACConfig, DataLogger | None], MFACController | MimoController]

_CONTROLLER_REGISTRY: dict[tuple[bool, str], _ControllerCtor] = {
    (True, "CFDL"): CFDLController,
    (True, "PFDL"): PFDLController,
    (True, "FFDL"): FFDLController,
    (False, "CFDL"): MimoCfdlController,
    (False, "PFDL"): MimoPfdlController,
    (False, "FFDL"): MimoFfdlController,
}


def create_controller(
    config: MFACConfig, logger: DataLogger | None = None
) -> MFACController | MimoController:
    """根据 ``MFACConfig.controller`` 与 ``dim`` 字段创建对应格式的控制器.

    参数:
        config: 包含 ``controller`` 字段的配置对象。
        logger: 可选的数据记录器；省略时若 ``config.enable_logging`` 为真
            则自动创建默认记录器。

    返回:
        与配置匹配的控制器实例。

    异常:
        ValueError: 当 ``controller`` 字段不是 CFDL/PFDL/FFDL 时抛出。
    """
    key = (config.dim == 1, config.controller)
    cls = _CONTROLLER_REGISTRY.get(key)
    if cls is None:
        raise ValueError(f"未知的控制器类型: {config.controller!r}, dim={config.dim}")
    return cls(config, logger)
