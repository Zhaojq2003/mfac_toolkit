# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""DataLogger 与控制器记录功能的测试."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import pytest
import yaml
from mfac_toolkit import CFDLController, FFDLController, MFACConfig, PFDLController
from mfac_toolkit.logger import DataLogger


def _load_csv_rows(run_dir: Path) -> tuple[list[str], list[dict[str, str]]]:
    """读取运行目录下的 data.csv，返回表头与数据行."""
    data_path = run_dir / "data.csv"
    with data_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    return fieldnames, rows


def test_disabled_logger_does_not_create_files(tmp_path: Path) -> None:
    """关闭时即使调用 log_step 也不应创建任何文件."""
    logger = DataLogger(enabled=False, log_dir=tmp_path)
    logger.set_metadata(test="disabled")
    logger.log_step(step=0, y=0.0)
    logger.close()

    assert list(tmp_path.iterdir()) == []


def test_enabled_logger_creates_metadata_and_csv(tmp_path: Path) -> None:
    """启用时应创建带 metadata.yaml 与 data.csv 的运行子目录."""
    with DataLogger(enabled=True, log_dir=tmp_path, metadata={"run": "unit"}) as logger:
        logger.log_step(step=0, y=0.0, u=0.1, phi=0.5)
        logger.log_step(step=1, y=0.2, u=0.2, phi=0.6)

    run_dirs = [d for d in tmp_path.iterdir() if d.is_dir()]
    assert len(run_dirs) == 1
    run_dir = run_dirs[0]

    metadata_path = run_dir / "metadata.yaml"
    assert metadata_path.exists()
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata: dict[str, Any] = yaml.safe_load(file)
    assert metadata["run"] == "unit"
    assert "timestamp" in metadata

    fieldnames, rows = _load_csv_rows(run_dir)
    assert fieldnames == ["step", "y", "u", "phi"]
    assert len(rows) == 2
    assert rows[0]["step"] == "0"
    assert rows[1]["phi"] == "0.6"


def test_cfdl_logs_per_step(tmp_path: Path) -> None:
    """CFDL 控制器启用记录后，每一步 update 都应在 data.csv 中生成一行."""
    config = MFACConfig(enable_logging=True, log_dir=str(tmp_path))
    controller = CFDLController(config)

    for _ in range(5):
        controller.update(y=0.1, yd=1.0)

    if controller.logger is not None:
        controller.logger.close()

    run_dirs = [d for d in tmp_path.iterdir() if d.is_dir()]
    assert len(run_dirs) == 1
    fieldnames, rows = _load_csv_rows(run_dirs[0])

    assert "step" in fieldnames
    assert "y" in fieldnames
    assert "yd" in fieldnames
    assert "u" in fieldnames
    assert "u_prev" in fieldnames
    assert "y_prev" in fieldnames
    assert "phi" in fieldnames
    assert "delta_y" in fieldnames
    assert "delta_u" in fieldnames
    assert "delta_u_new" in fieldnames
    assert "e" in fieldnames
    assert len(rows) == 5

    metadata_path = run_dirs[0] / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)
    assert metadata["controller"] == "CFDLController"
    assert metadata["controller_format"] == "CFDL"
    assert metadata["config"]["eta"] == pytest.approx(config.eta)


def test_pfdl_logs_vector_phi(tmp_path: Path) -> None:
    """PFDL 控制器应将 phi 向量展开为 phi_0、phi_1 … 列."""
    config = MFACConfig(L_y=0, L_u=2, enable_logging=True, log_dir=str(tmp_path))
    controller = PFDLController(config)

    controller.update(y=0.1, yd=1.0)
    controller.update(y=0.2, yd=1.0)

    if controller.logger is not None:
        controller.logger.close()

    run_dirs = [d for d in tmp_path.iterdir() if d.is_dir()]
    fieldnames, rows = _load_csv_rows(run_dirs[0])

    assert "phi_0" in fieldnames
    assert "phi_1" in fieldnames
    assert "u_prev" in fieldnames
    assert "y_prev" in fieldnames
    assert "delta_u_new" in fieldnames
    assert len(rows) == 2

    metadata_path = run_dirs[0] / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)
    assert metadata["controller_format"] == "PFDL"


def test_ffdl_logs_vector_phi(tmp_path: Path) -> None:
    """FFDL 控制器应将 phi 向量展开为 phi_0、phi_1 … 列."""
    config = MFACConfig(L_y=1, L_u=2, enable_logging=True, log_dir=str(tmp_path))
    controller = FFDLController(config)

    controller.update(y=0.1, yd=1.0)
    controller.update(y=0.2, yd=1.0)
    controller.update(y=0.3, yd=1.0)

    if controller.logger is not None:
        controller.logger.close()

    run_dirs = [d for d in tmp_path.iterdir() if d.is_dir()]
    fieldnames, rows = _load_csv_rows(run_dirs[0])

    assert "phi_0" in fieldnames
    assert "phi_1" in fieldnames
    assert "phi_2" in fieldnames
    assert "u_prev" in fieldnames
    assert "y_prev" in fieldnames
    assert "delta_u_new" in fieldnames
    assert len(rows) == 3

    metadata_path = run_dirs[0] / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)
    assert metadata["controller_format"] == "FFDL"


def test_config_metadata_is_serializable(tmp_path: Path) -> None:
    """Metadata 中可直接放入 MFACConfig 实例，最终能正确 YAML 序列化."""
    cfg = MFACConfig(eta=0.9, mu=2.0)
    with DataLogger(enabled=True, log_dir=tmp_path) as logger:
        logger.set_metadata(config=cfg)
        logger.log_step(step=0, y=0.0)

    run_dirs = [d for d in tmp_path.iterdir() if d.is_dir()]
    metadata_path = run_dirs[0] / "metadata.yaml"
    with metadata_path.open("r", encoding="utf-8") as file:
        metadata = yaml.safe_load(file)

    assert metadata["config"]["eta"] == pytest.approx(0.9)
    assert metadata["config"]["mu"] == pytest.approx(2.0)
