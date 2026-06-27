# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 包冒烟测试."""

from __future__ import annotations

import numpy as np
import pytest
from pydantic import ValidationError

from mfac_toolkit import CFDLController, FFDLController, MFACConfig, PFDLController
from mfac_toolkit.analysis import iae, overshoot, pseudo_frequency_response, rmse
from mfac_toolkit.model import NonlinearDiscretePlant, StateSpacePlant


def test_config_validation() -> None:
    """非法参数应抛出 ValidationError."""
    with pytest.raises(ValidationError):
        MFACConfig(eta=0.0)
    with pytest.raises(ValidationError):
        MFACConfig(mu=-1.0)
    with pytest.raises(ValidationError):
        MFACConfig(rho=1.5)
    with pytest.raises(ValidationError):
        MFACConfig(lambda_=-0.1)


def test_config_yaml_roundtrip(tmp_path) -> None:
    """配置应能经 YAML 往返后保持一致."""
    cfg = MFACConfig(eta=0.9, mu=2.0, rho=0.5, lambda_=0.2)
    path = tmp_path / "config.yaml"
    cfg.to_yaml(path)
    loaded = MFACConfig.from_yaml(path)
    assert loaded == cfg


def test_nonlinear_plant() -> None:
    """非线性被控对象应按确定性方程推进."""
    plant = NonlinearDiscretePlant(y0=0.0)
    y1 = plant.update(0.5)
    assert y1 == pytest.approx(0.125)


def test_state_space_plant() -> None:
    """一阶离散被控对象的动态应与预期一致."""
    a = 0.9
    b = 0.1
    plant = StateSpacePlant(A=np.array([[a]]), B=np.array([[b]]), C=np.array([[1.0]]))
    y = plant.update(1.0)
    assert y == pytest.approx(b)


def test_cfdl_step() -> None:
    """控制器应返回有限控制信号."""
    ctrl = CFDLController(MFACConfig())
    u = ctrl.update(y=0.0, yd=1.0)
    assert np.isfinite(u)


def test_metrics() -> None:
    """完美跟踪时指标应为零."""
    y = np.array([0.0, 0.5, 1.0, 1.0])
    yd = y.copy()
    t = np.array([0.0, 0.1, 0.2, 0.3])
    assert iae(y, yd, t) == pytest.approx(0.0, abs=1e-12)
    assert rmse(y, yd) == pytest.approx(0.0, abs=1e-12)
    assert overshoot(y, yd) == pytest.approx(0.0, abs=1e-12)


def test_frequency_response() -> None:
    """频率响应应返回单边谱."""
    phi = np.sin(np.linspace(0.0, 4.0 * np.pi, 128))
    freq, mag, phase = pseudo_frequency_response(phi, dt=0.01)
    assert freq.shape == mag.shape == phase.shape
    assert freq[0] == pytest.approx(0.0)


def test_config_order_validation() -> None:
    """伪阶数参数应在越界时抛出 ValidationError."""
    with pytest.raises(ValidationError):
        MFACConfig(L_y=-1)
    with pytest.raises(ValidationError):
        MFACConfig(L_u=0)


def test_pfdl_step() -> None:
    """PFDL 控制器应返回有限控制信号."""
    ctrl = PFDLController(MFACConfig(L_y=0, L_u=2))
    u = ctrl.update(y=0.0, yd=1.0)
    assert np.isfinite(u)
    assert ctrl.get_phi().shape == (2,)


def test_ffdl_step() -> None:
    """FFDL 控制器应返回有限控制信号."""
    ctrl = FFDLController(MFACConfig(L_y=1, L_u=2))
    u = ctrl.update(y=0.0, yd=1.0)
    assert np.isfinite(u)
    assert ctrl.get_phi().shape == (3,)


def test_pfdl_recovers_cfdl_when_lu_one() -> None:
    """PFDL 在 L_u=1 时应退化为 CFDL 行为."""
    cfg = MFACConfig(L_y=0, L_u=1)
    cfdl = CFDLController(cfg)
    pfdl = PFDLController(cfg)
    for _ in range(10):
        y = 0.1
        yd = 1.0
        assert cfdl.update(y, yd) == pytest.approx(pfdl.update(y, yd))


def test_ffdl_recovers_pfdl_when_ly_zero() -> None:
    """FFDL 在 L_y=0 时应与 PFDL 等价."""
    cfg = MFACConfig(L_y=0, L_u=2)
    pfdl = PFDLController(cfg)
    ffdl = FFDLController(cfg)
    for _ in range(10):
        y = 0.1
        yd = 1.0
        assert pfdl.update(y, yd) == pytest.approx(ffdl.update(y, yd))
