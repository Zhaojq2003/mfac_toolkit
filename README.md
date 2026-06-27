# MFAC Toolkit

基于**紧格式（CFDL）、偏格式（PFDL）与全格式（FFDL）动态线性化**的**无模型自适应控制（MFAC）**生产级 Python 工具包，面向 SISO 离散时间系统。

## 特性

- 支持 CFDL、PFDL、FFDL 三种动态线性化格式。
- 核心控制循环使用 Rust 实现，Python 侧为薄包装，兼顾性能与易用性。
- 职责清晰分离：
  - `CFDLController.update(y, yd)` / `PFDLController.update(...)` / `FFDLController.update(...)` 仅返回下一时刻控制输入。
  - 分析与可视化模块只消费外部收集的 NumPy 数组。
  - `DataLogger` 可选地记录每步元数据与控制数据。
- 完整的 Google Style 中文注释与类型注解。
- YAML 格式的控制器配置。
- 包含非线性离散被控对象与状态空间被控对象示例，以及可运行的仿真脚本。
- 性能指标：IAE、ITAE、ISE、RMSE、超调量、调节时间。
- PPD 序列的伪频率响应分析。
- 参数扫描热力图与等高线图。

## 安装

### 方式一：安装预编译 wheel（推荐）

对外发布版本通过 GitHub Releases 提供预编译的 x86_64 manylinux wheel：

```bash
pip install https://github.com/Zhaojq2003/mfac_toolkit/releases/download/v1.0.0/mfac_toolkit-1.0.0-cp312-cp312-manylinux_2_34_x86_64.whl
```

请将上方 URL 替换为实际 Release 中的 wheel 文件名。

### 方式二：从源码构建

本仓库仅包含 Python 源码与预编译的 Rust 扩展。若希望从源码构建 Rust 扩展，请先在私有仓库 `mfac_core` 中完成构建，再将生成的 `.so` 文件复制到 `mfac_toolkit/` 下。

推荐使用 [uv](https://docs.astral.sh/uv/) 管理依赖：

```bash
uv sync --extra dev
uv pip install -e .
```

## 快速开始

```python
import numpy as np
from mfac_toolkit import CFDLController, MFACConfig
from mfac_toolkit.model import NonlinearDiscretePlant

# 被控对象与控制器
plant = NonlinearDiscretePlant(y0=0.0)
ctrl = CFDLController(MFACConfig(rho=0.1, lambda_=0.02))

# 外部仿真循环负责收集历史数据
n_steps = 200
t = np.arange(n_steps) * 0.02
yd = np.ones(n_steps)  # 阶跃参考
y = np.zeros(n_steps)
u = np.zeros(n_steps)

for k in range(n_steps - 1):
    y[k] = plant.y
    u[k] = ctrl.update(y=y[k], yd=yd[k])
    plant.update(u[k])

y[-1] = plant.y
```

更完整的参考跟踪示例（含指标、绘图与日志）见 `examples/simulation_example.py`。

## YAML 配置

控制器参数可通过 YAML 文件配置，并用 `MFACConfig.from_yaml()` 加载：

```python
from mfac_toolkit import MFACConfig

cfg = MFACConfig.from_yaml("examples/config.yaml")
ctrl = CFDLController(cfg)
```

`examples/config.yaml` 提供了完整可调参数示例，所有字段均为可选，缺失字段将使用默认值。核心字段说明如下：

| 字段 | 含义 | 默认值 | 约束 |
|------|------|--------|------|
| `controller` | 控制器格式 | `"CFDL"` | 可选 `CFDL` / `PFDL` / `FFDL` |
| `eta` | PPD 投影算法学习率 η | `1.0` | `0 < eta <= 2` |
| `mu` | 投影算法正则化项 μ | `1.0` | 必须为正 |
| `rho` | 控制律步长因子 ρ | `0.1` | `0 < rho <= 1` |
| `lambda_` | 控制增量加权系数 λ | `0.02` | 必须为正 |
| `eps` | PPD 估计重置阈值 ε | `1e-5` | 必须为正 |
| `L_y` | 输出历史长度（FFDL 伪阶数） | `0` | CFDL/PFDL 必须为 0 |
| `L_u` | 输入历史长度（伪阶数） | `1` | 必须 >= 1 |
| `initial_phi` | PPD 估计初值 φ̂(0) | `0.5` | 标量，决定控制方向 |
| `u0` | 初始控制输入 | `0.0` | — |
| `u_min` / `u_max` | 控制输入饱和限 | `null` | 可选，须满足 `u_min <= u_max` |
| `enable_logging` | 是否记录每步数据 | `false` | — |
| `log_dir` | 日志根目录 | `"log"` | 相对路径 |

配置文件支持 Pydantic 校验：非法取值、未知键或 `u_min > u_max` 等错误会在加载时直接抛出。

## 常用命令

| 任务 | 命令 |
|------|------|
| 同步依赖 | `uv sync --extra dev` |
| 运行测试 | `env -u PYTHONPATH uv run --extra dev pytest -v` |
| 运行示例 | `uv run python examples/simulation_example.py` |
| 代码检查 | `uv run ruff check .` |
| 类型检查 | `uv run mypy mfac_toolkit` |
| 构建 wheel | `uv build` |

## 运行测试

```bash
# 推荐：隔离外部 PYTHONPATH（如 ROS）
env -u PYTHONPATH uv run --extra dev pytest -v
```

当前共 48 个 Python 测试，预期全部通过。

## 项目结构

```
.
├── mfac_toolkit/              # Python 包
│   ├── __init__.py
│   ├── config.py              # MFACConfig 数据类，参数校验与 YAML 序列化
│   ├── model.py               # 被控对象仿真器
│   ├── controller.py          # CFDLController、PFDLController、FFDLController（Rust 包装）
│   ├── logger.py              # DataLogger：可选启用的运行元信息与每步数据记录
│   ├── tuning.py              # 初值整定与参数映射工具
│   ├── _mfac_core.pyi         # Rust 扩展类型存根
│   ├── _mfac_core*.so         # 预编译 Rust 扩展
│   ├── py.typed               # PEP 561 类型标记
│   ├── analysis/
│   │   ├── metrics.py
│   │   └── frequency.py
│   └── visualization/
│       ├── time_response.py
│       └── sweep.py
├── tests/
└── examples/
```

## 闭源分发说明

本仓库对外公开 Python 工具包源码，核心 Rust 算法源码位于私有仓库 `mfac_core`，不对外公开。

对外发布版本通过 GitHub Releases 提供预编译的 x86_64 manylinux wheel：

```bash
pip install https://github.com/Zhaojq2003/mfac_toolkit/releases/download/v1.0.0/mfac_toolkit-1.0.0-cp312-cp312-manylinux_2_34_x86_64.whl
```

请将上方 URL 替换为实际 Release 中的 wheel 文件名。当前编译产物为 CPython 3.12、x86_64 平台。

## 许可证

Proprietary - RobotX. All rights reserved.
