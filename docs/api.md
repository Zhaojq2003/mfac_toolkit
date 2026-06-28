# API 速查表

`mfac-toolkit` 的公开接口集中在 `mfac_toolkit` 包下。本文档按使用频率排列，适合发布版本时快速查阅。

## 配置：MFACConfig

`MFACConfig` 是所有控制器的统一超参数配置，支持代码构造与 YAML 读写。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `controller` | `str` | `"CFDL"` | 控制器格式：`CFDL` / `PFDL` / `FFDL` |
| `eta` | `float` | `1.0` | PPD 投影算法学习率，需满足 `0 < eta <= 2` |
| `mu` | `float` | `1.0` | 投影算法分母正则化项，必须为正 |
| `rho` | `float` | `0.1` | 控制律步长因子，需满足 `0 < rho <= 1` |
| `lambda_` | `float` | `0.02` | 控制增量加权系数，必须为正 |
| `eps` | `float` | `1e-5` | PPD 估计值与控制增量重置阈值 |
| `L_y` | `int` | `0` | 输出历史长度（FFDL 伪阶数） |
| `L_u` | `int` | `1` | 输入历史长度（伪阶数），至少为 `1` |
| `initial_phi` | `float` | `0.5` | PPD 估计初始值，标量将广播到所有通道 |
| `u0` | `float` | `0.0` | 初始控制输入 |
| `u_min` | `float \| None` | `None` | 控制输入可选下限 |
| `u_max` | `float \| None` | `None` | 控制输入可选上限 |
| `enable_logging` | `bool` | `False` | 是否在运行时记录每步数据 |
| `log_dir` | `str` | `"log"` | 日志保存的根目录 |

```python
from mfac_toolkit import MFACConfig

config = MFACConfig(controller="PFDL", L_u=3, rho=0.5)
config = MFACConfig.from_yaml("config.yaml")
config.to_yaml("config.yaml")
```

## 控制器

| 类 | 说明 | 伪阶数要求 |
|----|------|------------|
| `CFDLController` | 紧格式动态线性化 | `L_y = 0`，`L_u >= 1` |
| `PFDLController` | 偏格式动态线性化 | `L_y = 0`，`L_u >= 1` |
| `FFDLController` | 全格式动态线性化 | `L_u >= 1` |

统一接口：

```python
u = controller.update(y=current_output, yd=desired_output)  # 返回下一时刻控制输入
controller.reset()
phi = controller.get_phi()
controller.set_phi_hat([0.5])
```

`FFDLController` 额外支持 per-component 步长因子：

```python
controller.set_rho_vector([0.1, 0.2, ...])  # 长度必须为 L_y + L_u
```

## 工厂函数

```python
from mfac_toolkit import create_controller

controller = create_controller(config)  # 根据 config.controller 自动选择类型
```

## 数据记录：DataLogger

```python
from mfac_toolkit import DataLogger

with DataLogger(enabled=True, log_dir="log") as logger:
    logger.log_step(step=0, y=0.0, yd=1.0, u=0.1)
    print(logger.current_run_dir)
```

首次调用 `log_step` 时，会在 `log_dir/<timestamp>/` 下创建运行子目录，写入 `metadata.yaml`，并以 CSV 格式追加每步数据到 `data.csv`。

## 初值整定

### PFDL 伪梯度初值

```python
from mfac_toolkit import (
    pid_to_pfdl,
    zn_response_to_pfdl,
    critical_proportional_to_pfdl,
    apply_pfdl_initial_guess,
)

psi = pid_to_pfdl(kp=1.0, ti=1.0, td=0.0, ts=0.01, order=3)
apply_pfdl_initial_guess(controller, psi)
```

### FFDL 步长因子整定

```python
from mfac_toolkit import apply_ffdl_zn_tuning, apply_ffdl_critical_tuning

apply_ffdl_zn_tuning(controller, k=1.0, tau=1.0, time_delay=0.1, ts=0.01)
apply_ffdl_critical_tuning(controller, ku=10.0, tu=2.0, ts=0.01)
```

## 类型约定

- `y`、`yd`、`u` 均为标量 `float`。
- `phi` 在 Python 层以 `numpy.ndarray` 暴露：CFDL 形状为 `(1,)`，PFDL 为 `(L_u,)`，FFDL 为 `(L_y + L_u,)`。
- `MFACConfig` 为不可变对象（Pydantic `frozen=True`）。
