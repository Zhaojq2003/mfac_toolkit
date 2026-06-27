# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""mfac_toolkit.tuning 模块单元测试."""

from __future__ import annotations

import numpy as np
import pytest

from mfac_toolkit import FFDLController, MFACConfig, PFDLController
from mfac_toolkit.tuning import (
    apply_ffdl_critical_tuning,
    apply_ffdl_zn_tuning,
    apply_pfdl_initial_guess,
    critical_proportional_to_pfdl,
    ffdl_critical_tuning,
    ffdl_zn_tuning,
    physical_pfdl_params,
    pid_to_pfdl,
    zn_response_to_pfdl,
)


def test_pid_to_pfdl() -> None:
    """格式 A Le=3 的计算结果应符合给定公式."""
    psi = pid_to_pfdl(kp=2.0, ti=1.0, td=0.5, ts=0.1)
    assert psi.shape == (3,)
    assert psi[0] == pytest.approx(-0.2)
    assert psi[1] == pytest.approx(12.0)
    assert psi[2] == pytest.approx(-10.0)


def test_pid_to_pfdl_order_1() -> None:
    """格式 A Le=1 应只返回 -Kp."""
    psi = pid_to_pfdl(kp=2.0, ti=1.0, td=0.5, ts=0.1, order=1)
    assert psi.shape == (1,)
    assert psi[0] == pytest.approx(-2.0)


def test_pid_to_pfdl_order_2() -> None:
    """格式 A Le=2 应符合 PI 映射."""
    psi = pid_to_pfdl(kp=2.0, ti=1.0, td=0.5, ts=0.1, order=2)
    assert psi.shape == (2,)
    assert psi[0] == pytest.approx(-0.2)
    assert psi[1] == pytest.approx(12.0)


def test_pid_to_pfdl_zero_derivative() -> None:
    """Td=0 时 Le=3 应得到 ψ3=0."""
    psi = pid_to_pfdl(kp=1.0, ti=1.0, td=0.0, ts=0.1)
    assert psi[2] == pytest.approx(0.0)


def test_zn_response_to_pfdl() -> None:
    """格式 B Le=3 的计算结果应符合给定公式."""
    # a = K * lambda / tau = 1 * 0.5 / 2 = 0.25
    psi = zn_response_to_pfdl(k=1.0, tau=2.0, time_delay=0.5, ts=0.1)
    assert psi.shape == (3,)
    assert psi[0] == pytest.approx(-0.48)
    assert psi[1] == pytest.approx(16.8)
    assert psi[2] == pytest.approx(-12.0)


def test_zn_response_to_pfdl_order_variants() -> None:
    """格式 B 应支持 Le=1/2/3."""
    for order in (1, 2, 3):
        psi = zn_response_to_pfdl(k=1.0, tau=2.0, time_delay=0.5, ts=0.1, order=order)
        assert psi.shape == (order,)


def test_critical_proportional_to_pfdl() -> None:
    """格式 C Le=3 的计算结果应符合给定公式."""
    psi = critical_proportional_to_pfdl(ku=4.0, tu=2.0, ts=0.1)
    assert psi.shape == (3,)
    assert psi[0] == pytest.approx(-0.24)
    assert psi[1] == pytest.approx(8.4)
    assert psi[2] == pytest.approx(-6.0)


def test_critical_proportional_to_pfdl_order_variants() -> None:
    """格式 C 应支持 Le=1/2/3."""
    for order in (1, 2, 3):
        psi = critical_proportional_to_pfdl(ku=4.0, tu=2.0, ts=0.1, order=order)
        assert psi.shape == (order,)


def test_apply_pfdl_initial_guess() -> None:
    """注入初值后估计器应使用指定向量."""
    psi = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    ctrl = PFDLController(MFACConfig(L_y=0, L_u=3))
    apply_pfdl_initial_guess(ctrl, psi)

    np.testing.assert_array_almost_equal(ctrl.get_phi(), psi)

    # reset 后仍应恢复到注入的初值
    ctrl.reset()
    np.testing.assert_array_almost_equal(ctrl.get_phi(), psi)


def test_apply_pfdl_initial_guess_different_orders() -> None:
    """不同 Le 的 PFDL 控制器应能注入对应长度的初值."""
    for order in (1, 2, 3):
        psi = np.ones(order, dtype=np.float64) * 0.7
        ctrl = PFDLController(MFACConfig(L_y=0, L_u=order))
        apply_pfdl_initial_guess(ctrl, psi)
        np.testing.assert_array_almost_equal(ctrl.get_phi(), psi)


def test_apply_pfdl_initial_guess_wrong_length() -> None:
    """向量长度与控制器伪阶数不匹配时应抛出 ValueError."""
    ctrl = PFDLController(MFACConfig(L_y=0, L_u=2))
    with pytest.raises(ValueError):
        apply_pfdl_initial_guess(ctrl, [1.0, 2.0, 3.0])

    ctrl3 = PFDLController(MFACConfig(L_y=0, L_u=3))
    with pytest.raises(ValueError):
        apply_pfdl_initial_guess(ctrl3, [1.0, 2.0])


def test_ffdl_zn_tuning_pi() -> None:
    """FFDL (1,1) Z-N 整定应返回长度 2 的 rho 向量."""
    ctrl = FFDLController(MFACConfig(L_y=1, L_u=1, lambda_=0.1))
    rho = ffdl_zn_tuning(ctrl, k=1.0, tau=2.0, time_delay=0.5, ts=0.1)
    assert rho.shape == (2,)
    assert np.all(np.isfinite(rho))


def test_ffdl_zn_tuning_pid() -> None:
    """FFDL (2,1) Z-N 整定应返回长度 3 的 rho 向量."""
    ctrl = FFDLController(MFACConfig(L_y=2, L_u=1, lambda_=0.1))
    rho = ffdl_zn_tuning(ctrl, k=1.0, tau=2.0, time_delay=0.5, ts=0.1)
    assert rho.shape == (3,)
    assert np.all(np.isfinite(rho))


def test_ffdl_critical_tuning_pi() -> None:
    """FFDL (1,1) 临界比例度整定应返回长度 2 的 rho 向量."""
    ctrl = FFDLController(MFACConfig(L_y=1, L_u=1, lambda_=0.1))
    rho = ffdl_critical_tuning(ctrl, ku=4.0, tu=2.0, ts=0.1)
    assert rho.shape == (2,)
    assert np.all(np.isfinite(rho))


def test_ffdl_critical_tuning_pid() -> None:
    """FFDL (2,1) 临界比例度整定应返回长度 3 的 rho 向量."""
    ctrl = FFDLController(MFACConfig(L_y=2, L_u=1, lambda_=0.1))
    rho = ffdl_critical_tuning(ctrl, ku=4.0, tu=2.0, ts=0.1)
    assert rho.shape == (3,)
    assert np.all(np.isfinite(rho))


def test_apply_ffdl_zn_tuning_sets_rho_vector() -> None:
    """apply_ffdl_zn_tuning 应将 rho 向量写入控制律."""
    ctrl = FFDLController(MFACConfig(L_y=1, L_u=1, lambda_=0.1))
    apply_ffdl_zn_tuning(ctrl, k=1.0, tau=2.0, time_delay=0.5, ts=0.1)
    assert ctrl.rho_vector is not None
    assert ctrl.rho_vector.shape == (2,)


def test_apply_ffdl_critical_tuning_sets_rho_vector() -> None:
    """apply_ffdl_critical_tuning 应将 rho 向量写入控制律."""
    ctrl = FFDLController(MFACConfig(L_y=2, L_u=1, lambda_=0.1))
    apply_ffdl_critical_tuning(ctrl, ku=4.0, tu=2.0, ts=0.1)
    assert ctrl.rho_vector is not None
    assert ctrl.rho_vector.shape == (3,)


def test_ffdl_tuning_unsupported_order() -> None:
    """不支持的 (L_y, L_u) 组合应抛出 ValueError."""
    ctrl = FFDLController(MFACConfig(L_y=1, L_u=2))
    with pytest.raises(ValueError):
        ffdl_zn_tuning(ctrl, k=1.0, tau=2.0, time_delay=0.5, ts=0.1)


def test_physical_pfdl_params_shape() -> None:
    """physical_pfdl_params 应返回正确长度的 phi0 与合法配置."""
    phi0, cfg = physical_pfdl_params(
        inertia=0.02,
        dt=0.001,
        kp=5.0,
        ki=0.01,
        kd=0.1,
        u_max=5.0,
        order=2,
    )
    assert phi0.shape == (2,)
    assert cfg.L_u == 2
    assert cfg.u_min == -5.0
    assert cfg.u_max == 5.0
    assert 0.0 < cfg.rho <= 1.0
    assert cfg.lambda_ > 0.0
    assert cfg.mu > 0.0


def test_physical_pfdl_params_order_1() -> None:
    """order=1 时 phi0 长度应为 1."""
    phi0, cfg = physical_pfdl_params(
        inertia=0.02,
        dt=0.001,
        kp=5.0,
        ki=0.01,
        kd=0.1,
        u_max=5.0,
        order=1,
    )
    assert phi0.shape == (1,)
    assert cfg.L_u == 1


def test_physical_pfdl_params_invalid_inputs() -> None:
    """非法输入应抛出 ValueError."""
    with pytest.raises(ValueError):
        physical_pfdl_params(inertia=-1.0, dt=0.001, kp=5.0, ki=0.01, kd=0.1, u_max=5.0)
    with pytest.raises(ValueError):
        physical_pfdl_params(inertia=0.02, dt=-0.001, kp=5.0, ki=0.01, kd=0.1, u_max=5.0)
    with pytest.raises(ValueError):
        physical_pfdl_params(inertia=0.02, dt=0.001, kp=-5.0, ki=0.01, kd=0.1, u_max=5.0)
    with pytest.raises(ValueError):
        physical_pfdl_params(inertia=0.02, dt=0.001, kp=5.0, ki=0.01, kd=-0.1, u_max=5.0)


@pytest.mark.parametrize(
    ("func", "kwargs"),
    [
        (pid_to_pfdl, {"kp": 1.0, "ti": -1.0, "td": 0.0, "ts": 0.1}),
        (pid_to_pfdl, {"kp": 1.0, "ti": 1.0, "td": -0.1, "ts": 0.1}),
        (pid_to_pfdl, {"kp": 1.0, "ti": 1.0, "td": 0.0, "ts": 0.0}),
        (pid_to_pfdl, {"kp": 1.0, "ti": 1.0, "td": 0.0, "ts": 0.1, "order": 4}),
        (zn_response_to_pfdl, {"k": -1.0, "tau": 1.0, "time_delay": 0.1, "ts": 0.1}),
        (zn_response_to_pfdl, {"k": 1.0, "tau": 0.0, "time_delay": 0.1, "ts": 0.1}),
        (zn_response_to_pfdl, {"k": 1.0, "tau": 1.0, "time_delay": 0.0, "ts": 0.1}),
        (critical_proportional_to_pfdl, {"ku": 0.0, "tu": 1.0, "ts": 0.1}),
        (critical_proportional_to_pfdl, {"ku": 1.0, "tu": -1.0, "ts": 0.1}),
    ],
)
def test_invalid_parameters(
    func: callable,
    kwargs: dict[str, float],
) -> None:
    """非法参数应抛出 ValueError."""
    with pytest.raises(ValueError):
        func(**kwargs)
