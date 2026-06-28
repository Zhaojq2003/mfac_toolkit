# mfac-toolkit 使用教程

本文档覆盖 `mfac-toolkit` 的公开接口。假设已安装 Python 3.12+：

```bash
pip install mfac-toolkit
```

## 快速开始

```python
from mfac_toolkit import MFACConfig, create_controller

config = MFACConfig.from_yaml("mfac_toolkit/examples/siso_config.yaml")
controller = create_controller(config)

u = controller.update(y=0.0, yd=1.0)  # 当前输出、期望输出 -> 下一时刻控制输入
```

## 配置

`MFACConfig` 是基于 Pydantic 的不可变配置类，支持 YAML 读写与参数校验。

```python
config = MFACConfig(controller="PFDL", L_u=3, rho=0.5)
config = MFACConfig.from_yaml("config.yaml")
config.to_yaml("config.yaml")
```

### 字段说明

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `controller` | `"CFDL"` | 控制器格式：`CFDL` / `PFDL` / `FFDL` |
| `dim` | `1` | 系统维度，`1` 为 SISO，`>=2` 为 MIMO |
| `eta` | `1.0` | PPD 投影算法学习率 `η`，`0 < eta <= 2` |
| `mu` | `1.0` | 投影算法分母正则化项 `μ`，必须为正 |
| `rho` | `0.1` | 控制律步长因子 `ρ`，`0 < rho <= 1` |
| `lambda_` | `0.02` | 控制增量加权系数 `λ`，必须为正 |
| `eps` | `1e-5` | PPD 估计值与控制增量重置阈值 |
| `L_y` | `0` | 输出历史长度（FFDL 伪阶数），CFDL/PFDL 须为 `0` |
| `L_u` | `1` | 输入历史长度（伪阶数），须 `>= 1` |
| `initial_phi` | `0.5` | PPD/PJM 估计初始值；标量广播，也支持数组 |
| `u0` | `0.0` | 初始控制输入 |
| `u_min` | `None` | 控制输入下限（可选） |
| `u_max` | `None` | 控制输入上限（可选）；同时设置时须 `u_min <= u_max` |
| `m_upper` | `1.0e6` | MIMO PJM 范数上界 |
| `m_lower` | `1.0e-6` | MIMO PJM 范数下界 |
| `enable_logging` | `False` | 是否记录每步数据 |
| `log_dir` | `"log"` | 日志保存根目录 |

`MimoConfig` 已弃用，请改用 `MFACConfig(dim=...)`。

## 控制器

`create_controller(config)` 根据 `controller` 与 `dim` 自动选择控制器。

| 控制器 | 类型 | 伪阶数要求 |
|--------|------|------------|
| `CFDLController` | SISO 紧格式 | `L_y = 0`，`L_u >= 1` |
| `PFDLController` | SISO 偏格式 | `L_y = 0`，`L_u >= 1` |
| `FFDLController` | SISO 全格式 | `L_u >= 1` |
| `MimoCfdlController` | MIMO 紧格式 | `L_y = 0`，`L_u = 1` |
| `MimoPfdlController` | MIMO 偏格式 | `L_y = 0`，`L_u >= 1` |
| `MimoFfdlController` | MIMO 全格式 | `L_u >= 1` |

统一接口：

```python
u = controller.update(y, yd)  # 返回下一时刻控制输入
controller.reset()              # 重置状态
phi = controller.get_phi()      # 当前 PPD/PJM 估计
controller.set_phi_hat(phi)     # 手动设置估计值
```

MIMO 下 `y`、`yd`、`u` 均为形状 `(dim,)` 的 `numpy.ndarray`。

## `initial_phi` 形状

标量会自动广播；数组形式用于为不同历史分量或通道设置差异化初值。

### SISO（`dim = 1`）

| 控制器 | 形状 | 示例 |
|--------|------|------|
| `CFDLController` | 标量 或 `(1,)` | `0.5` 或 `[0.5]` |
| `PFDLController` | 标量 或 `(L_u,)` | `0.5` 或 `[0.5, 0.3, 0.2]` |
| `FFDLController` | 标量 或 `(L_y + L_u,)` | `0.5` 或 `[0.1, 0.3, 0.5]` |

### MIMO（`dim >= 2`）

| 控制器 | 形状 | YAML 示例 |
|--------|------|-----------|
| `MimoCfdlController` | 标量 或 `(dim, dim)` | `0.5` 或 `[[0.5, 0.0], [0.0, 0.5]]` |
| `MimoPfdlController` | 标量 或 `(L_u, dim, dim)` | `0.5` 或 `L_u` 个 `dim x dim` 矩阵 |
| `MimoFfdlController` | 标量 或 `(L_y + L_u, dim, dim)` | 同上，矩阵个数为 `L_y + L_u` |

## FFDL 分量步长

`FFDLController` 支持按分量设置 `rho`：

```python
controller.set_rho_vector([0.1, 0.2, ...])  # 长度须为 L_y + L_u
```

## 数据记录

`DataLogger` 可选启用。启用后会在 `log_dir/<timestamp>/` 下创建运行目录，写入 `metadata.yaml`，并以 CSV 追加每步数据到 `data.csv`。

```python
from mfac_toolkit import DataLogger

with DataLogger(enabled=True, log_dir="log") as logger:
    logger.log_step(step=0, y=0.0, yd=1.0, u=0.1)
    print(logger.current_run_dir)
```

设置 `config.enable_logging = True` 时，控制器会自动创建默认记录器。

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

## YAML 示例

SISO 配置（`siso_config.yaml`）：

```yaml
controller: "CFDL"
dim: 1
eta: 1.0
mu: 1.0
rho: 0.1
lambda_: 0.02
eps: 1.0e-5
L_y: 0
L_u: 1
initial_phi: 0.5
u0: 0.0
u_min: null
u_max: null
enable_logging: false
log_dir: "log"
```

MIMO 配置（`mimo_config.yaml`）：

```yaml
controller: "PFDL"
dim: 2
L_y: 0
L_u: 2
initial_phi:
  - [[0.5, 0.0],
     [0.0, 0.5]]
  - [[0.3, 0.0],
     [0.0, 0.3]]
m_upper: 1.0e6
m_lower: 1.0e-6
```

## 类型约定

- `MFACConfig` 不可变（`frozen=True`）。
- SISO 下 `y`、`yd`、`u` 为 `float`；MIMO 下为形状 `(dim,)` 的 `numpy.ndarray`。
- `phi` 形状：
  - SISO：CFDL `(1,)`，PFDL `(L_u,)`，FFDL `(L_y + L_u,)`
  - MIMO：CFDL `(dim, dim)`，PFDL `(L_u, dim, dim)`，FFDL `(L_y + L_u, dim, dim)`
