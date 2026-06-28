# mfac-toolkit

<p align="center">
  <strong>面向 SISO/MIMO 离散时间系统的无模型自适应控制 Python 工具包</strong>
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

config = MFACConfig.from_yaml("mfac_toolkit/examples/siso_config.yaml")
controller = create_controller(config)

u = controller.update(y=0.0, yd=1.0)  # 当前输出、期望输出 → 下一时刻控制输入
```

运行内置示例：

```bash
python -m mfac_toolkit.examples.basic_example
```

## 配置字段说明

`MFACConfig` 支持从 YAML 加载，所有字段均有默认值。下表列出常用字段及其含义：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `controller` | `"CFDL"` | 控制器格式：`CFDL` / `PFDL` / `FFDL` |
| `dim` | `1` | 系统维度，`1` 为 SISO，`>=2` 为 MIMO |
| `eta` | `1.0` | PPD 投影算法学习率 `η`，需满足 `0 < eta <= 2` |
| `mu` | `1.0` | 投影算法分母正则化项 `μ`，必须为正 |
| `rho` | `0.1` | 控制律步长因子 `ρ`，需满足 `0 < rho <= 1` |
| `lambda_` | `0.02` | 控制增量加权系数 `λ`，必须为正 |
| `eps` | `1e-5` | PPD 估计值与控制增量重置阈值 `ε` |
| `L_y` | `0` | 输出历史长度（FFDL 伪阶数），CFDL/PFDL 必须为 `0` |
| `L_u` | `1` | 输入历史长度（伪阶数），必须 `>= 1` |
| `initial_phi` | `0.5` | PPD/PJM 估计初始值；标量广播，也支持数组 |
| `u0` | `0.0` | 初始控制输入 |
| `u_min` | `null` | 控制输入下限（可选） |
| `u_max` | `null` | 控制输入上限（可选）；若同时设置须满足 `u_min <= u_max` |
| `m_upper` | `1.0e6` | MIMO PJM 范数上界 |
| `m_lower` | `1.0e-6` | MIMO PJM 范数下界 |
| `enable_logging` | `false` | 是否记录每步运行数据 |
| `log_dir` | `"log"` | 日志保存根目录 |

完整字段说明、使用示例与 phi 形状参考见 [TUTORIAL.md](./TUTORIAL.md)。

## 功能

- 支持 **CFDL / PFDL / FFDL** 三种动态线性化格式
- 支持 SISO 与 MIMO 系统，统一接口：`controller.update(y, yd)`
- YAML 配置与参数校验
- 可选的仿真数据记录

## 示例

- `mfac_toolkit.examples.basic_example`：SISO 闭环仿真，对应 `mfac_toolkit/examples/siso_config.yaml`
- `mfac_toolkit.examples.mimo_example`：MIMO 闭环仿真，对应 `mfac_toolkit/examples/mimo_config.yaml`
- `mfac_toolkit.examples.mimo_coupled_example`：MIMO 耦合系统解耦仿真，对应 `mfac_toolkit/examples/mimo_config.yaml`
- `mfac_toolkit.examples.plants`：示例被控对象

## 文档

- [英文教程 / Tutorial](./TUTORIAL.md)
- [更新日志](./CHANGELOG.md)

## 许可证

本项目采用 [MIT 许可证](LICENSE) 开源。

MIT License  
Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology)  
Author: Jiqian Zhao <zhaojq2003@163.com>
