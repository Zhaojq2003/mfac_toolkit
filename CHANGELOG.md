# Changelog

本文件遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 格式。

## [Unreleased]

## [1.1.1] - 2026-06-28

### Changed

- 将 `controller.py` 拆分为 `controller/` 子包：`_base.py`、`_siso.py`、`_mimo.py`，公开接口保持不变。

### Removed

- 删除已弃用的 `MimoConfig`，请改用 `MFACConfig(dim=...)`。

## [1.1.0] - 2026-06-28

### Added

- 支持 MIMO 系统：新增 `MimoCfdlController`、`MimoPfdlController`、`MimoFfdlController`。
- `MFACConfig` 新增 `dim`、`m_upper`、`m_lower` 字段，通过 `dim` 切换 SISO/MIMO。
- 新增 MIMO 示例：`mimo_example.py`、`mimo_coupled_example.py`、`mimo_config.yaml`。
- 新增 `tests/test_mimo.py` 覆盖 MIMO 配置与控制器接口。
- 新增根目录 `TUTORIAL.md` 教程文档。

### Deprecated

- `MimoConfig` 已弃用，请改用 `MFACConfig(dim=...)`。

## [1.0.2] - 2026-06-28

### Added

- 新增 pytest 测试，覆盖配置校验、数据记录器、控制器 reset/phi 维度、整定函数。
- Release CI 新增 `checks` job，在构建 wheel 前运行 ruff、mypy、pytest。

### Fixed

- `FFDLController.reset()` 现在会清空 `rho_vector`。
- 删除 `CFDLController` 中未使用的 `u_prev2` 状态。
- 控制器 `reset()` 现在会显式将 `phi_hat` 恢复到 `initial_phi`。

## [1.0.1] - 2026-06-27

### Added

- 初始发布：支持 **CFDL / PFDL / FFDL** 三种动态线性化格式。
- `MFACConfig`：基于 Pydantic 的 YAML/代码配置与参数校验。
- `CFDLController`、`PFDLController`、`FFDLController` 与 `create_controller` 工厂函数。
- `DataLogger`：可选的 CSV + YAML 运行数据记录。
- `tuning` 模块：PID / Z-N 响应曲线法 / 临界比例度法到 PFDL/FFDL 的初值映射。
- 预编译 Rust 扩展 `_mfac_core`，支持 Linux x86_64 与 Windows AMD64。
