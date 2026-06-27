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

直接安装最新版本：

```bash
pip install https://github.com/Zhaojq2003/mfac_toolkit/archive/refs/heads/main.zip
```

> 也可以从 [GitHub Releases](https://github.com/Zhaojq2003/mfac_toolkit/releases/latest) 下载最新 `.whl` 后运行 `pip install ./<下载的 whl 文件>`。

本地开发可使用：

```bash
uv sync --extra dev
uv pip install -e .
```

## 快速开始 / Quick Start

查看 `examples/` 目录下的可运行示例：

- `examples/basic_example.py`：最小 CFDL 闭环仿真，注释中标注了扩展用法
- `examples/config.yaml`：YAML 配置示例
- `examples/plants.py`：示例用离散被控对象

运行示例：

```bash
uv run python examples/basic_example.py
```

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
    ├── basic_example.py
    ├── config.yaml
    └── plants.py
```

## 许可证 / License

本项目采用 [MIT 许可证](LICENSE) 开源。

MIT License  
Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology)  
Author: RobotX 实验室 (RobotX Lab) <zhaojq2003@163.com>
