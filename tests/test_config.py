# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>

"""MFACConfig 参数校验与 YAML 读写测试."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from pydantic import ValidationError

from mfac_toolkit import MFACConfig


def test_defaults() -> None:
    """默认配置应符合文档约定."""
    cfg = MFACConfig()
    assert cfg.controller == "CFDL"
    assert cfg.eta == 1.0
    assert cfg.mu == 1.0
    assert cfg.rho == 0.1
    assert cfg.L_u == 1


def test_controller_validation() -> None:
    """非法 controller 字符串应触发校验错误."""
    with pytest.raises(ValidationError):
        MFACConfig(controller="INVALID")


def test_eta_bounds() -> None:
    """Eta 必须满足 0 < eta <= 2."""
    with pytest.raises(ValidationError):
        MFACConfig(eta=0.0)
    with pytest.raises(ValidationError):
        MFACConfig(eta=3.0)


def test_saturation_bounds() -> None:
    """u_min 不能大于 u_max."""
    with pytest.raises(ValidationError):
        MFACConfig(u_min=10.0, u_max=0.0)


def test_yaml_round_trip() -> None:
    """to_yaml / from_yaml 应保持配置一致."""
    cfg = MFACConfig(controller="PFDL", L_u=3, rho=0.5)
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "cfg.yaml"
        cfg.to_yaml(path)
        loaded = MFACConfig.from_yaml(path)
        assert loaded == cfg
