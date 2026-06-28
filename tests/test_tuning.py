# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>

"""PFDL/FFDL 初值整定接口测试.

这些测试需要编译后的 `_mfac_core` 扩展。
"""

from __future__ import annotations

import numpy as np

from mfac_toolkit import (
    MFACConfig,
    PFDLController,
    apply_pfdl_initial_guess,
    critical_proportional_to_pfdl,
    pid_to_pfdl,
    zn_response_to_pfdl,
)


def test_pid_to_pfdl_shape() -> None:
    """pid_to_pfdl 返回向量长度应等于 order."""
    psi = pid_to_pfdl(1.0, 1.0, 0.0, 0.01, order=3)
    assert psi.shape == (3,)


def test_zn_response_to_pfdl_shape() -> None:
    """zn_response_to_pfdl 返回向量长度应等于 order."""
    psi = zn_response_to_pfdl(1.0, 1.0, 0.1, 0.01, order=2)
    assert psi.shape == (2,)


def test_critical_proportional_to_pfdl_shape() -> None:
    """critical_proportional_to_pfdl 返回向量长度应等于 order."""
    psi = critical_proportional_to_pfdl(10.0, 2.0, 0.01, order=1)
    assert psi.shape == (1,)


def test_apply_pfdl_initial_guess() -> None:
    """注入的伪梯度初值应能被 get_phi 读出."""
    controller = PFDLController(MFACConfig(controller="PFDL", L_u=3))
    apply_pfdl_initial_guess(controller, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(controller.get_phi(), [0.1, 0.2, 0.3], atol=1e-6)
