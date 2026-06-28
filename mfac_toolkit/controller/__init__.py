# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-28

"""MFAC 控制器包：编译扩展的薄包装."""

from __future__ import annotations

from collections.abc import Callable

from mfac_toolkit.config import MFACConfig
from mfac_toolkit.controller._base import MFACController, MimoController
from mfac_toolkit.controller._mimo import (
    MimoCfdlController,
    MimoFfdlController,
    MimoPfdlController,
)
from mfac_toolkit.controller._siso import CFDLController, FFDLController, PFDLController
from mfac_toolkit.logger import DataLogger

__all__ = [
    "MFACController",
    "MimoController",
    "CFDLController",
    "PFDLController",
    "FFDLController",
    "MimoCfdlController",
    "MimoPfdlController",
    "MimoFfdlController",
    "create_controller",
]

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
