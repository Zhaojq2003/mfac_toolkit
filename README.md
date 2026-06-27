# MFAC Toolkit

<p align="center">
  <strong>基于紧格式 / 偏格式 / 全格式动态线性化的无模型自适应控制 Python 工具包</strong><br>
  <strong>A Python toolkit for Model-Free Adaptive Control (MFAC) based on CFDL / PFDL / FFDL dynamic linearization</strong><br>
  <em>面向 SISO 离散时间系统 / For SISO discrete-time systems</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.12%2B-blue" alt="Python 3.12+">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License: MIT">
</p>

---

## 特性 / Features

- 支持 **CFDL**、**PFDL**、**FFDL** 三种动态线性化格式。
- 高性能 Rust 编译扩展，Python 侧为薄包装，兼顾性能与易用性。
- 控制器接口极简：`controller.update(y, yd)` 只根据当前输出 `y` 与期望输出 `yd` 返回下一时刻控制输入 `u`。
- YAML 格式的控制器配置，Pydantic 参数校验。
- 示例中提供非线性离散被控对象与状态空间被控对象，以及可运行的仿真脚本。
- `DataLogger` 可选地记录每步元数据与控制数据。

## 安装 / Installation

### 方式一：安装预编译 wheel（推荐）

从 [GitHub Releases](https://github.com/Zhaojq2003/mfac_toolkit/releases) 下载对应 Python 版本与平台的 wheel，然后本地安装：

```bash
pip install ./mfac_toolkit-*.whl
```

Release 页面会提供形如 `mfac_toolkit-<version>-cp312-cp312-manylinux_2_34_x86_64.whl` 的预编译包。若后续发布到 PyPI，也可直接：

```bash
pip install mfac-toolkit
```

### 方式二：从源码构建

本仓库包含 Python 源码与预编译扩展。如需本地使用，直接安装即可：

```bash
uv sync --extra dev
uv pip install -e .
```

## 快速开始 / Quick Start

```python
import numpy as np
from mfac_toolkit import CFDLController, MFACConfig
from mfac_toolkit.examples.plants import NonlinearDiscretePlant

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

更完整的参考跟踪示例见 `examples/simulation_example.py`，初值整定示例见 `examples/tuning_example.py`。

## 常用命令 / Common Commands

| 任务 | 命令 |
|------|------|
| 同步依赖 | `uv sync --extra dev` |
| 运行示例 | `uv run python examples/simulation_example.py` |
| 代码检查 | `uv run ruff check .` |
| 类型检查 | `uv run mypy .` |
| 构建 wheel | `uv build` |

## 项目结构 / Project Structure

```
.
├── mfac_toolkit/       # Python 包
│   ├── __init__.py
│   ├── config.py
│   ├── controller.py
│   ├── logger.py
│   ├── tuning.py
│   ├── _mfac_core.pyi
│   ├── _mfac_core*.so
│   └── py.typed
└── examples/
    ├── config.yaml
    ├── plants.py
    ├── simulation_example.py
    └── tuning_example.py
```

## 许可证 / License

本项目采用 [MIT 许可证](LICENSE) 开源。

MIT License  
Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology)  
Author: RobotX 实验室 (RobotX Lab) <zhaojq2003@163.com>
