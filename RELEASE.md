# mfac_toolkit 1.0.0 发布手册

Copyright (c) 2026 RobotX. All rights reserved.  
Author: Jiqian Zhao <zhaojq2003@163.com>

## 版本与兼容性

- Python 版本要求：Python 3.12+
- 预编译 wheel 平台：x86_64 manylinux
- 推荐安装方式：`pip install <wheel URL>`

## 本地构建 wheel

```bash
uv build
```

构建出的 wheel 包含 Python 源码与预编译 Rust 扩展，产物位于 `dist/`。

## GitHub Release 发布步骤

1. 确认 `mfac_toolkit/_mfac_core*.so` 为最新编译版本（在私有 `mfac_core` 仓库中构建）。
2. 确认 `uv run ruff check .` 与 `uv run mypy .` 通过。
3. 创建并推送 tag：

   ```bash
   git tag v1.0.0
   git push origin v1.0.0
   ```

4. 在 GitHub Releases 页面基于该 tag 创建 Release。
5. 上传 `dist/` 中的 wheel 文件作为 release asset。
6. 用户通过以下命令安装：

   ```bash
   pip install https://github.com/Zhaojq2003/mfac_toolkit/releases/download/v1.0.0/mfac_toolkit-1.0.0-cp312-cp312-manylinux_2_34_x86_64.whl
   ```

## PyPI 发布步骤（可选）

```bash
uv publish dist/*.whl
```

建议仅上传 wheel，不上传 sdist。
