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

从 PyPI 直接安装（会根据你的操作系统和架构自动选择预编译 wheel）：

```bash
pip install mfac-toolkit
```

> 也可以从 [GitHub Releases](https://github.com/Zhaojq2003/mfac_toolkit/releases/latest) 下载对应平台的 `.whl` 后运行 `pip install ./<下载的 whl 文件>`。

## 快速开始 / Quick Start

查看包内示例：

- `mfac_toolkit.examples.basic_example`：最小 CFDL 闭环仿真
- `mfac_toolkit.examples.config`：YAML 配置示例
- `mfac_toolkit.examples.plants`：示例用离散被控对象

安装后可直接运行：

```bash
python -m mfac_toolkit.examples.basic_example
```

本地源码开发时（已安装可编辑模式）：

```bash
uv run python -m mfac_toolkit.examples.basic_example
```

## 常用命令 / Common Commands

| 任务 | 命令 |
|------|------|
| 同步依赖 | `uv sync --extra dev` |
| 运行示例 | `uv run python -m mfac_toolkit.examples.basic_example` |
| 代码检查 | `uv run ruff check .` |
| 类型检查 | `uv run mypy .` |
| 开发模式构建 | `uv run maturin develop` |
| 构建 release wheel | `uv run maturin build --release` |

## 项目结构 / Project Structure

```
.
├── mfac_toolkit/           # Python 包
│   ├── __init__.py
│   ├── config.py
│   ├── controller.py
│   ├── logger.py
│   ├── tuning.py
│   ├── _mfac_core.pyi
│   ├── py.typed
│   └── examples/
│       ├── basic_example.py
│       ├── config.yaml
│       └── plants.py
└── pyproject.toml
```

## 跨平台 Wheel 构建 / Cross-Platform Wheels

`mfac_toolkit` 使用 [maturin](https://github.com/PyO3/maturin) 编译 Rust 扩展，并通过 [cibuildwheel](https://github.com/pypa/cibuildwheel) 在 GitHub Actions 中生成以下平台的 wheel：

- Linux: `x86_64`、`aarch64`（manylinux2014）
- macOS: `x86_64`、`arm64`、`universal2`
- Windows: `AMD64`

打 `v*` tag 时会自动触发构建并发布到 PyPI。

## 许可证 / License

本项目采用 [MIT 许可证](LICENSE) 开源。

MIT License  
Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology)  
Author: RobotX 实验室 (RobotX Lab) <zhaojq2003@163.com>
