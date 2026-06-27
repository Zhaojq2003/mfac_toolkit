# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""利用离散傅里叶变换对 PPD 序列进行频域分析."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def pseudo_frequency_response(
    phi_sequence: NDArray[np.float64] | np.ndarray,
    dt: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    r"""从 PPD 时间序列计算伪频率响应.

    将 PPD 序列 :math:`\\hat{\\phi}(k)` 视为时变增益，其单边幅值谱可反映
    自适应动态中占主导的频率分量。由于被控对象非线性且时变，这不是真正
    的频率响应，但可作为有用的诊断工具。

    参数:
        phi_sequence: PPD 估计序列，形状 ``(N,)``。
        dt: 采样周期，必须为正。

    返回:
        单边谱三元组 ``(freq, magnitude, phase)``。若 ``dt`` 无量纲，
        ``freq`` 单位为 rad/sample；若 ``dt`` 以秒为单位，则单位为 rad/s。

    异常:
        ValueError: 当 ``dt`` 非正或序列长度不足时抛出。
    """
    phi = np.asarray(phi_sequence, dtype=float).reshape(-1)
    n = phi.size
    if n < 2:
        raise ValueError("phi_sequence 至少包含两个样本")
    if dt <= 0.0:
        raise ValueError("dt 必须为正")

    spectrum = np.fft.rfft(phi)
    magnitude = np.abs(spectrum)
    phase = np.angle(spectrum)
    freq = np.fft.rfftfreq(n, d=dt) * 2.0 * np.pi  # 角频率
    return freq, magnitude, phase
