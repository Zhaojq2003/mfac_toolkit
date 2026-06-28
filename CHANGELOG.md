# Changelog

本文件遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 格式。

## [Unreleased]

## [1.0.1] - 2026-06-27

### Added

- 初始发布：支持 **CFDL / PFDL / FFDL** 三种动态线性化格式。
- `MFACConfig`：基于 Pydantic 的 YAML/代码配置与参数校验。
- `CFDLController`、`PFDLController`、`FFDLController` 与 `create_controller` 工厂函数。
- `DataLogger`：可选的 CSV + YAML 运行数据记录。
- `tuning` 模块：PID / Z-N 响应曲线法 / 临界比例度法到 PFDL/FFDL 的初值映射。
- 预编译 Rust 扩展 `_mfac_core`，支持 Linux x86_64 与 Windows AMD64。
