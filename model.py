# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""用于 MFAC 控制器测试的离散时间被控对象仿真模型."""

from __future__ import annotations

from typing import Protocol

import numpy as np
from numpy.typing import NDArray


class Plant(Protocol):
    """离散时间 SISO 被控对象协议."""

    def reset(self) -> None:
        """重置被控对象到初始状态."""

    def update(self, u: float) -> float:
        """推进被控对象一个采样步，并返回新的输出.

        参数:
            u: 当前步施加的控制输入。

        返回:
            下一步的被控对象输出。
        """


class NonlinearDiscretePlant:
    r"""经典非线性离散时间基准被控对象.

    动力学方程为

    .. math::
        y(k+1) = \\frac{y(k)}{1 + y(k)^2} + u(k)^3

    该对象在 MFAC 文献中广泛用于展示未知非线性系统的数据驱动控制效果。

    属性:
        y0: 初始输出。
        y: 当前输出 ``y(k)``。

    示例:
        >>> plant = NonlinearDiscretePlant(y0=0.0)
        >>> y = plant.update(0.1)
    """

    def __init__(self, y0: float = 0.0) -> None:
        """初始化被控对象.

        参数:
            y0: 初始输出值。
        """
        self.y0: float = y0
        self.y: float = y0

    def reset(self) -> None:
        """将被控对象输出重置为 ``y0``."""
        self.y = self.y0

    def update(self, u: float) -> float:
        """推进被控对象一个采样步.

        参数:
            u: 当前步的控制输入。

        返回:
            下一步的被控对象输出。
        """
        y_next = self.y / (1.0 + self.y * self.y) + u * u * u
        self.y = y_next
        return float(y_next)


class StateSpacePlant:
    r"""线性时不变离散时间状态空间被控对象.

    动力学方程为

    .. math::
        x(k+1) &= A x(k) + B u(k) \\
        y(k)   &= C x(k) + D u(k)

    所有矩阵向量运算均使用 NumPy 向量化执行。

    属性:
        A: 状态转移矩阵。
        B: 输入矩阵。
        C: 输出矩阵。
        D: 直馈矩阵。
        x: 当前状态向量。
    """

    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        C: NDArray[np.float64],
        D: NDArray[np.float64] | None = None,
        x0: NDArray[np.float64] | None = None,
    ) -> None:
        """初始化状态空间被控对象.

        参数:
            A: 状态转移矩阵，形状 ``(n, n)``。
            B: 输入矩阵，形状 ``(n, 1)`` 或 ``(n,)``。
            C: 输出矩阵，形状 ``(1, n)`` 或 ``(n,)``。
            D: 可选直馈矩阵，形状 ``(1, 1)`` 或标量。
            x0: 可选初始状态向量，形状 ``(n,)``。

        异常:
            ValueError: 当矩阵形状不兼容时抛出。
        """
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
        """将状态重置为 ``x0``."""
        self.x = self.x0.copy()

    def update(self, u: float) -> float:
        """推进被控对象一个采样步.

        参数:
            u: 当前步的控制输入。

        返回:
            下一步的被控对象输出。
        """
        self._u_vec[0, 0] = float(u)
        self.x = self.A @ self.x + (self.B @ self._u_vec).ravel()
        y = (self.C @ self.x.reshape(-1, 1) + self.D @ self._u_vec).item()
        return float(y)
