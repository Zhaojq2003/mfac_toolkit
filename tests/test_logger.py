# Copyright (c) 2026 北方工业大学 RobotX 实验室 (RobotX Lab, North China University of Technology).
# Author: Jiqian Zhao <zhaojq2003@163.com>

"""DataLogger 文件记录测试."""

from __future__ import annotations

import csv
import tempfile
from pathlib import Path

import yaml

from mfac_toolkit import DataLogger


def test_disabled_logger_does_nothing() -> None:
    """关闭状态下记录不应创建目录或文件."""
    logger = DataLogger(enabled=False)
    logger.log_step(step=0, y=1.0)
    assert logger.current_run_dir is None


def test_context_manager_creates_files() -> None:
    """启用后应创建 metadata.yaml 与 data.csv."""
    with tempfile.TemporaryDirectory() as tmp:
        logger = DataLogger(enabled=True, log_dir=tmp)
        with logger:
            logger.set_metadata(controller="CFDL")
            logger.log_step(step=0, y=0.0, yd=1.0, u=0.1)
            run_dir = logger.current_run_dir

        assert run_dir is not None
        assert run_dir.parent == Path(tmp)

        metadata_path = run_dir / "metadata.yaml"
        data_path = run_dir / "data.csv"
        assert metadata_path.exists()
        assert data_path.exists()

        with data_path.open(newline="") as file:
            rows = list(csv.DictReader(file))
            assert len(rows) == 1
            assert rows[0]["y"] == "0.0"

        with metadata_path.open(encoding="utf-8") as file:
            meta = yaml.safe_load(file)
            assert meta["controller"] == "CFDL"
