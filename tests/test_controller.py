# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>

"""控制器工厂、状态维度与 reset 行为测试.

这些测试需要编译后的 `_mfac_core` 扩展；在 CI 与本地 maturin 开发构建后可运行。
"""

from __future__ import annotations

import numpy as np
import pytest

from mfac_toolkit import MFACConfig, create_controller
from mfac_toolkit.controller import CFDLController, FFDLController, PFDLController


def test_create_controller_selects_class() -> None:
    """工厂函数应根据 config.controller 返回对应类型."""
    assert isinstance(create_controller(MFACConfig(controller="CFDL")), CFDLController)
    assert isinstance(create_controller(MFACConfig(controller="PFDL", L_u=2)), PFDLController)
    assert isinstance(
        create_controller(MFACConfig(controller="FFDL", L_y=1, L_u=2)), FFDLController
    )


def test_invalid_controller_raises() -> None:
    """非法 controller 类型应在工厂中抛出 ValueError."""
    with pytest.raises(ValueError):
        create_controller(MFACConfig(controller="BAD"))


def test_cfdl_phi_shape() -> None:
    """CFDL 的 phi 应为长度 1 的向量."""
    controller = CFDLController(MFACConfig(controller="CFDL", L_u=1))
    assert controller.get_phi().shape == (1,)


def test_pfdl_phi_shape_and_reset() -> None:
    """PFDL 的 phi 形状应与 L_u 一致，reset 后回到 initial_phi."""
    cfg = MFACConfig(controller="PFDL", L_u=3)
    controller = PFDLController(cfg)
    assert controller.get_phi().shape == (3,)

    controller.set_phi_hat([0.1, 0.2, 0.3])
    np.testing.assert_array_equal(controller.get_phi(), [0.1, 0.2, 0.3])

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), [cfg.initial_phi] * 3)


def test_ffdl_rho_vector_reset() -> None:
    """FFDL reset 后 Python 层 rho_vector 应被清空."""
    controller = FFDLController(MFACConfig(controller="FFDL", L_y=1, L_u=2))
    controller.set_rho_vector([0.1, 0.2, 0.3])
    assert controller.rho_vector is not None

    controller.reset()
    assert controller.rho_vector is None


def test_pfdl_initial_phi_vector() -> None:
    """PFDL 使用向量 initial_phi 时 get_phi 形状与值应正确."""
    cfg = MFACConfig(controller="PFDL", L_u=3, initial_phi=[0.1, 0.2, 0.3])
    controller = PFDLController(cfg)
    np.testing.assert_array_equal(controller.get_phi(), [0.1, 0.2, 0.3])

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), [0.1, 0.2, 0.3])


def test_ffdl_initial_phi_vector() -> None:
    """FFDL 使用向量 initial_phi 时 get_phi 形状与值应正确."""
    phi = [0.1, 0.2, 0.3]
    cfg = MFACConfig(controller="FFDL", L_y=1, L_u=2, initial_phi=phi)
    controller = FFDLController(cfg)
    np.testing.assert_array_equal(controller.get_phi(), phi)

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), phi)
