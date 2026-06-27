# MFAC Toolkit

Copyright (c) 2026 RobotX. All rights reserved.  
Author: Jiqian Zhao <zhaojq2003@163.com>

基于**紧格式（CFDL）、偏格式（PFDL）与全格式（FFDL）动态线性化**的**无模型自适应控制（MFAC）**生产级 Python 工具包，面向 SISO 离散时间系统。

## 特性

- 支持 CFDL、PFDL、FFDL 三种动态线性化格式。
- 高性能编译扩展，Python 侧为薄包装，兼顾性能与易用性。
- `CFDLController.update(y, yd)` / `PFDLController.update(...)` / `FFDLController.update(...)` 仅返回下一时刻控制输入。
- YAML 格式的控制器配置，Pydantic 参数校验。
- 包含非线性离散被控对象与状态空间被控对象示例，以及可运行的仿真脚本。
- `DataLogger` 可选地记录每步元数据与控制数据。

## 安装

### 方式一：安装预编译 wheel（推荐）

对外发布版本通过 GitHub Releases 提供预编译的 x86_64 manylinux wheel：

```bash
pip install https://github.com/Zhaojq2003/mfac_toolkit/releases/download/v1.0.0/mfac_toolkit-1.0.0-cp312-cp312-manylinux_2_34_x86_64.whl
```

请将上方 URL 替换为实际 Release 中的 wheel 文件名。

### 方式二：从源码构建

本仓库包含 Python 源码与预编译扩展。如需本地使用，直接安装即可：

```bash
uv sync --extra dev
uv pip install -e .
```

## 快速开始

```python
import numpy as np
from mfac_toolkit import CFDLController, MFACConfig
from mfac_toolkit.model import NonlinearDiscretePlant

plant = NonlinearDiscretePlant(y0=0.0)
ctrl = CFDLController(MFACConfig(rho=0.1, lambda_=0.02))

n_steps = 200
t = np.arange(n_steps) * 0.02
yd = np.ones(n_steps)
y = np.zeros(n_steps)
u = np.zeros(n_steps)

for k in range(n_steps - 1):
    y[k] = plant.y
    u[k] = ctrl.update(y=y[k], yd=yd[k])
    plant.update(u[k])

y[-1] = plant.y
```

更完整的参考跟踪示例见 `examples/simulation_example.py`。

## 常用命令

| 任务 | 命令 |
|------|------|
| 同步依赖 | `uv sync --extra dev` |
| 运行示例 | `uv run python examples/simulation_example.py` |
| 代码检查 | `uv run ruff check .` |
| 类型检查 | `uv run mypy .` |
| 构建 wheel | `uv build` |

## 项目结构

```
.
├── mfac_toolkit/       # Python 包
│   ├── __init__.py
│   ├── config.py
│   ├── controller.py
│   ├── logger.py
│   ├── model.py
│   ├── tuning.py
│   ├── _mfac_core.pyi
│   ├── _mfac_core*.so
│   └── py.typed
└── examples/
```

## 许可证

Proprietary - RobotX. All rights reserved.
