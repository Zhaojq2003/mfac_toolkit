# Copyright (c) 2026 RobotX. All rights reserved.
# Author: Jiqian Zhao <zhaojq2003@163.com>
# Date: 2026-06-27

"""MFAC 仿真数据记录器.

``DataLogger`` 提供可选启用的文件记录功能：每个运行使用独立子目录，
在 ``metadata.yaml`` 中记录本次运行的元信息，在 ``data.csv`` 中逐行
记录每个控制步的数据。
"""

from __future__ import annotations

import csv
import dataclasses
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from types import TracebackType
from typing import Any, Self, TextIO

import yaml
from pydantic import BaseModel


class DataLogger:
    """MFAC 运行数据记录器.

    默认关闭，开启后会在首次 ``log_step`` 时创建 ``log_dir/<timestamp>/``
    子目录，写入 ``metadata.yaml``，随后以 CSV 格式追加每步数据。

    属性:
        enabled: 是否启用记录。
        log_dir: 日志根目录。

    示例:
        >>> with DataLogger(enabled=True) as logger:
        ...     logger.set_metadata(controller="MFAC", config={"eta": 1.0})
        ...     logger.log_step(step=0, y=0.0, yd=1.0, u=0.1, phi=0.5)
    """

    def __init__(
        self,
        enabled: bool = False,
        log_dir: str | Path = "log",
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """初始化记录器.

        参数:
            enabled: 是否启用文件记录。
            log_dir: 日志根目录，相对于当前工作目录。
            metadata: 要写入 ``metadata.yaml`` 的运行元信息。
        """
        self.enabled: bool = enabled
        self.log_dir: Path = Path(log_dir)

        self._metadata: dict[str, Any] = dict(metadata) if metadata else {}
        self._run_dir: Path | None = None
        self._data_path: Path | None = None
        self._data_file: TextIO | None = None
        self._writer: csv.DictWriter[str] | None = None
        self._fieldnames: tuple[str, ...] | None = None
        self._closed: bool = False

    def set_metadata(self, **kwargs: Any) -> None:
        """设置或更新运行元信息，并立即同步到已创建的目录."""
        self._metadata.update(kwargs)
        if self._run_dir is not None:
            self._write_metadata()

    def _to_serializable(self, value: Any) -> Any:
        """将常见不可 YAML/JSON 序列化的对象转为标准类型."""
        if isinstance(value, BaseModel):
            return self._to_serializable(value.model_dump())
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return self._to_serializable(dataclasses.asdict(value))
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, Mapping):
            return {str(k): self._to_serializable(v) for k, v in value.items()}
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
            return [self._to_serializable(v) for v in value]
        return value

    def _write_metadata(self) -> None:
        """将当前 metadata 写入 run_dir/metadata.yaml."""
        if self._run_dir is None:
            return
        metadata_path = self._run_dir / "metadata.yaml"
        serializable = self._to_serializable(self._metadata)
        with metadata_path.open("w", encoding="utf-8", newline="") as file:
            yaml.safe_dump(serializable, file, sort_keys=False, allow_unicode=True)

    def _make_run_dir(self) -> Path:
        """在 log_dir 下创建带时间戳且不冲突的运行子目录."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.log_dir / timestamp
        if run_dir.exists():
            suffix = 1
            while (candidate := self.log_dir / f"{timestamp}_{suffix:03d}").exists():
                suffix += 1
            run_dir = candidate
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir

    def _format_value(self, value: Any) -> Any:
        """将每步数据格式化为适合 CSV 写入的值."""
        if isinstance(value, float):
            return value
        if isinstance(value, int):
            return value
        if isinstance(value, bool):
            return value
        if value is None:
            return ""
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
            return ";".join(str(self._format_value(v)) for v in value)
        return str(value)

    def _initialize(self, row: dict[str, Any]) -> None:
        """首次记录时创建目录、metadata 与 CSV 文件."""
        self._run_dir = self._make_run_dir()
        self._data_path = self._run_dir / "data.csv"
        self._fieldnames = tuple(row.keys())

        if "timestamp" not in self._metadata:
            self._metadata["timestamp"] = datetime.now().isoformat()
        self._write_metadata()

        self._data_file = self._data_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(
            self._data_file,
            fieldnames=list(self._fieldnames),
            lineterminator="\n",
        )
        self._writer.writeheader()

    def log_step(self, **values: Any) -> None:
        """记录一个控制步的数据.

        参数:
            **values: 当前步的键值对，所有步的键应保持一致。
        """
        if not self.enabled or self._closed:
            return

        row = {str(k): self._format_value(v) for k, v in values.items()}

        if self._writer is None:
            self._initialize(row)

        assert self._writer is not None
        assert self._fieldnames is not None

        # 只允许初始化时确定的列，防止 CSV 结构不一致。
        safe_row = {name: row.get(name, "") for name in self._fieldnames}
        self._writer.writerow(safe_row)
        if self._data_file is not None:
            self._data_file.flush()

    def close(self) -> None:
        """关闭已打开的文件."""
        if self._data_file is not None:
            self._data_file.close()
            self._data_file = None
        self._writer = None
        self._closed = True

    @property
    def current_run_dir(self) -> Path | None:
        """返回当前运行子目录，仅在首次 ``log_step`` 后有效."""
        return self._run_dir

    def __enter__(self) -> Self:
        """进入上下文，返回记录器自身."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        """退出上下文时关闭文件."""
        self.close()
