# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 控制器配置数据类."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class MFACConfig(BaseModel):
    """MFAC 控制器的超参数.

    属性:
        controller: 控制器格式，可选 "CFDL"、"PFDL"、"FFDL"。
        eta: PPD 投影算法的学习率，需满足 0 < eta <= 2。
        mu: 投影算法分母中的正则化项，必须为正。
        rho: 控制律步长因子，需满足 0 < rho <= 1。
        lambda_: 控制增量加权系数，必须为正。
        eps: PPD 估计值与控制增量重置阈值。
        L_y: 输出历史长度（FFDL 伪阶数），CFDL/PFDL 取 0。
        L_u: 输入历史长度（CFDL/PFDL/FFDL 伪阶数），至少为 1。
        initial_phi: PPD 估计的初始值与重置值，标量将广播到所有通道。
        u0: 初始控制输入。
        u_min: 控制输入可选的下限饱和值。
        u_max: 控制输入可选的上限饱和值。
        enable_logging: 是否在控制器运行时记录每步数据。
        log_dir: 日志保存的根目录（相对路径字符串）。

    异常:
        ValidationError: 当任一参数超出合法取值范围或 controller 类型非法时抛出。
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    controller: str = Field(default="CFDL")
    eta: float = Field(default=1.0, gt=0.0, le=2.0)
    mu: float = Field(default=1.0, gt=0.0)
    rho: float = Field(default=0.1, gt=0.0, le=1.0)
    lambda_: float = Field(default=0.02, gt=0.0)
    eps: float = Field(default=1e-5, gt=0.0)
    L_y: int = Field(default=0, ge=0)
    L_u: int = Field(default=1, ge=1)
    initial_phi: float = 0.5
    u0: float = 0.0
    u_min: float | None = None
    u_max: float | None = None
    enable_logging: bool = False
    log_dir: str = "log"

    @field_validator("controller")
    @classmethod
    def _check_controller(cls, value: str) -> str:
        """校验控制器格式."""
        allowed = {"CFDL", "PFDL", "FFDL"}
        if value not in allowed:
            raise ValueError(f"controller 必须是 {allowed} 之一，实际为 {value!r}")
        return value

    @model_validator(mode="after")
    def _check_saturation_bounds(self) -> MFACConfig:
        """校验控制输入上下限的交叉关系."""
        if self.u_min is not None and self.u_max is not None and self.u_min > self.u_max:
            raise ValueError("u_min 不能大于 u_max")
        return self

    @classmethod
    def from_yaml(cls, path: Path | str) -> Self:
        """从 YAML 文件加载配置.

        参数:
            path: YAML 配置文件路径。

        返回:
            由 YAML 内容构造的 ``MFACConfig`` 实例。

        异常:
            FileNotFoundError: 当 ``path`` 不存在时抛出。
            ValidationError: 当 YAML 中包含未知键或参数越界时抛出。
        """
        path = Path(path)
        with path.open("r", encoding="utf-8") as file:
            data: dict[str, Any] = yaml.safe_load(file) or {}

        return cls.model_validate(data)

    def to_yaml(self, path: Path | str) -> None:
        """将配置序列化为 YAML 文件.

        参数:
            path: 要写入的 YAML 文件路径。
        """
        path = Path(path)
        with path.open("w", encoding="utf-8") as file:
            yaml.safe_dump(self.model_dump(), file, sort_keys=False)
