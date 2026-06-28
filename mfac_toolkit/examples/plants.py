# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""示例用离散时间被控对象."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


class NonlinearDiscretePlant:
    r"""经典非线性离散时间基准被控对象.

    .. math::
        y(k+1) = \frac{y(k)}{1 + y(k)^2} + u(k)^3
    """

    def __init__(self, y0: float = 0.0) -> None:
        self.y0: float = y0
        self.y: float = y0

    def reset(self) -> None:
        """重置被控对象输出为 ``y0``."""
        self.y = self.y0

    def update(self, u: float) -> float:
        """推进被控对象一个采样步，返回下一步输出."""
        self.y = self.y / (1.0 + self.y * self.y) + u * u * u
        return float(self.y)


class StateSpacePlant:
    r"""线性时不变离散时间状态空间被控对象.

    .. math::
        x(k+1) &= A x(k) + B u(k) \\
        y(k)   &= C x(k) + D u(k)
    """

    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        C: NDArray[np.float64],
        D: NDArray[np.float64] | None = None,
        x0: NDArray[np.float64] | None = None,
    ) -> None:
        self.A = np.asarray(A, dtype=np.float64)
        self.B = np.asarray(B, dtype=np.float64)
        self.C = np.asarray(C, dtype=np.float64)
        self.D = np.asarray(D if D is not None else 0.0, dtype=np.float64)

        n = self.A.shape[0]
        if self.A.shape != (n, n):
            raise ValueError("A 必须为方阵")
        if self.B.shape not in ((n,), (n, 1)):
            raise ValueError("B 的形状必须为 (n,) 或 (n, 1)")
        if self.C.shape not in ((n,), (1, n)):
            raise ValueError("C 的形状必须为 (n,) 或 (1, n)")

        self.B = self.B.reshape(n, 1)
        self.C = self.C.reshape(1, n)
        self.D = self.D.reshape(1, 1) if np.ndim(self.D) == 0 else self.D.reshape(1, 1)

        self.x0: NDArray[np.float64] = np.zeros(n, dtype=np.float64) if x0 is None else np.asarray(x0, dtype=np.float64)
        if self.x0.shape != (n,):
            raise ValueError("x0 的形状必须为 (n,)")

        self.x: NDArray[np.float64] = self.x0.copy()
        self._u_vec: NDArray[np.float64] = np.empty((1, 1), dtype=np.float64)

    def reset(self) -> None:
        """重置状态为 ``x0``."""
        self.x = self.x0.copy()

    def update(self, u: float) -> float:
        """推进被控对象一个采样步，返回下一步输出."""
        self._u_vec[0, 0] = float(u)
        self.x = self.A @ self.x + (self.B @ self._u_vec).ravel()
        y = (self.C @ self.x.reshape(-1, 1) + self.D @ self._u_vec).item()
        return float(y)


class MimoLinearPlant:
    r"""MIMO 线性时不变静态增益被控对象.

    .. math::
        y(k+1) = G \, u(k)

    适用于演示 MIMO 控制器的解耦与跟踪效果。
    """

    def __init__(
        self,
        gain: NDArray[np.float64],
        y0: NDArray[np.float64] | None = None,
    ) -> None:
        """初始化静态增益矩阵与初始输出.

        参数:
            gain: 形状为 (dim, dim) 的增益矩阵 :math:`G`。
            y0: 初始输出向量，形状 (dim,)；默认为零向量。
        """
        self.gain = np.asarray(gain, dtype=np.float64)
        if self.gain.ndim != 2 or self.gain.shape[0] != self.gain.shape[1]:
            raise ValueError("gain 必须为方阵")

        self.dim = self.gain.shape[0]
        self.y0 = (
            np.zeros(self.dim, dtype=np.float64)
            if y0 is None
            else np.asarray(y0, dtype=np.float64)
        )
        if self.y0.shape != (self.dim,):
            raise ValueError("y0 形状必须为 (dim,)")

        self.y: NDArray[np.float64] = self.y0.copy()

    def reset(self) -> None:
        """重置输出为 ``y0``."""
        self.y = self.y0.copy()

    def update(self, u: NDArray[np.float64]) -> NDArray[np.float64]:
        """推进被控对象一个采样步，返回下一步输出向量."""
        u_arr = np.asarray(u, dtype=np.float64).reshape(-1)
        if u_arr.shape != (self.dim,):
            raise ValueError(f"u 形状 {u_arr.shape} ≠ ({self.dim},)")
        self.y = self.gain @ u_arr
        return self.y.copy()
