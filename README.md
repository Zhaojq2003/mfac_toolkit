# mfac-toolkit

<p align="center">
  <strong>面向 SISO 离散时间系统的无模型自适应控制 Python 工具包</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.12%2B-blue" alt="Python 3.12+">
  <img src="https://img.shields.io/badge/License-MIT-green" alt="License: MIT">
</p>

## 安装

**PyPI（推荐）**

```bash
pip install mfac-toolkit
```

**GitHub Release 下载 wheel**

从 [Releases](https://github.com/Zhaojq2003/mfac_toolkit/releases/latest) 下载对应平台的 `.whl` 后安装：

```bash
pip install ./<下载的 whl 文件>
```

**直接克隆使用**

仓库内已包含预编译扩展，克隆后即可安装使用：

```bash
git clone https://github.com/Zhaojq2003/mfac_toolkit.git
cd mfac_toolkit
pip install -e .
```

## 快速开始

```python
from mfac_toolkit import MFACConfig, create_controller

config = MFACConfig.from_yaml("config.yaml")
controller = create_controller(config)

u = controller.update(y=0.0, yd=1.0)  # 当前输出、期望输出 → 下一时刻控制输入
```

运行内置示例：

```bash
python -m mfac_toolkit.examples.basic_example
```

## 功能

- 支持 **CFDL / PFDL / FFDL** 三种动态线性化格式
- 极简控制器接口：`controller.update(y, yd)`
- YAML 配置与参数校验
- 可选的仿真数据记录

## 示例

- `mfac_toolkit.examples.basic_example`：CFDL 闭环仿真
- `mfac_toolkit.examples.config`：YAML 配置示例
- `mfac_toolkit.examples.plants`：示例被控对象

## 文档

- [API 速查表](./docs/api.md)
- [更新日志](./CHANGELOG.md)

## 许可证

本项目采用 [MIT 许可证](LICENSE) 开源。

MIT License  
Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology)  
Author: Jiqian Zhao <zhaojq2003@163.com>
