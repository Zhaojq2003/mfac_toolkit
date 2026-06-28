# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>

"""MIMO 控制器工厂、维度与收敛性测试.

这些测试需要编译后的 `_mfac_core` 扩展；在 CI 与本地 maturin 开发构建后可运行。
"""

from __future__ import annotations

import numpy as np
import pytest

from mfac_toolkit import MFACConfig, create_controller
from mfac_toolkit.controller import (
    MimoCfdlController,
    MimoFfdlController,
    MimoPfdlController,
)


def test_create_controller_selects_mimo_class() -> None:
    """工厂函数应根据 dim 返回 MIMO 控制器."""
    assert isinstance(create_controller(MFACConfig(dim=2, controller="CFDL")), MimoCfdlController)
    assert isinstance(
        create_controller(MFACConfig(dim=2, controller="PFDL", L_u=3)), MimoPfdlController
    )
    assert isinstance(
        create_controller(MFACConfig(dim=2, controller="FFDL", L_y=1, L_u=2)),
        MimoFfdlController,
    )


def test_mimo_cfdl_phi_shape_and_reset() -> None:
    """CFDL 的 phi 形状应为 (dim, dim)，reset 后回到 initial_phi."""
    cfg = MFACConfig(dim=2, controller="CFDL")
    controller = MimoCfdlController(cfg)
    assert controller.get_phi().shape == (2, 2)

    controller.set_phi_hat(np.array([[0.1, 0.2], [0.3, 0.4]]))
    np.testing.assert_array_equal(
        controller.get_phi(), [[0.1, 0.2], [0.3, 0.4]]
    )

    controller.reset()
    np.testing.assert_array_equal(
        controller.get_phi(), np.eye(2) * cfg.initial_phi
    )


def test_mimo_pfdl_phi_shape() -> None:
    """PFDL 的 phi 形状应为 (L_u, dim, dim)."""
    controller = MimoPfdlController(MFACConfig(dim=2, controller="PFDL", L_u=3))
    assert controller.get_phi().shape == (3, 2, 2)


def test_mimo_ffdl_phi_shape() -> None:
    """FFDL 的 phi 形状应为 (L_y + L_u, dim, dim)."""
    controller = MimoFfdlController(
        MFACConfig(dim=2, controller="FFDL", L_y=2, L_u=2)
    )
    assert controller.get_phi().shape == (4, 2, 2)


def test_mimo_cfdl_initial_phi_matrix() -> None:
    """MIMO-CFDL 使用矩阵 initial_phi 时应正确初始化."""
    phi = np.array([[0.1, 0.2], [0.3, 0.4]], dtype=np.float64)
    cfg = MFACConfig(dim=2, controller="CFDL", initial_phi=phi.tolist())
    controller = MimoCfdlController(cfg)
    np.testing.assert_array_equal(controller.get_phi(), phi)

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), phi)


def test_mimo_pfdl_initial_phi_3d() -> None:
    """MIMO-PFDL 使用 3D initial_phi 时应正确初始化."""
    phi = np.stack([np.eye(2) * (i + 1) for i in range(3)])
    cfg = MFACConfig(dim=2, controller="PFDL", L_u=3, initial_phi=phi.tolist())
    controller = MimoPfdlController(cfg)
    np.testing.assert_array_equal(controller.get_phi(), phi)

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), phi)


def test_mimo_ffdl_initial_phi_3d() -> None:
    """MIMO-FFDL 使用 3D initial_phi 时应正确初始化."""
    phi = np.stack([np.eye(2) * (i + 1) for i in range(4)])
    cfg = MFACConfig(dim=2, controller="FFDL", L_y=2, L_u=2, initial_phi=phi.tolist())
    controller = MimoFfdlController(cfg)
    np.testing.assert_array_equal(controller.get_phi(), phi)

    controller.reset()
    np.testing.assert_array_equal(controller.get_phi(), phi)


def test_mimo_cfdl_2x2_convergence() -> None:
    """MIMO-CFDL 应在 2×2 线性系统上收敛."""
    cfg = MFACConfig(dim=2, controller="CFDL", rho=0.5, lambda_=0.02)
    controller = MimoCfdlController(cfg)
    yd = np.array([1.0, 2.0])
    y = np.zeros(2)

    for _ in range(500):
        u = controller.update(y, yd)
        y[0] = 0.5 * u[0] + 0.3 * u[1]
        y[1] = 0.2 * u[0] + 0.6 * u[1]

    assert np.allclose(y, yd, atol=0.05), f"最终输出: {y}"


def test_invalid_controller_raises() -> None:
    """非法 controller 类型应在工厂中抛出 ValueError."""
    with pytest.raises(ValueError):
        create_controller(MFACConfig(dim=2, controller="BAD"))
