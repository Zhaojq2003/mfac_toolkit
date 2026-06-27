# mfac_toolkit 1.0.0 发布手册

## 版本与兼容性

- Python 版本要求：Python 3.12+
- 预编译 wheel 平台：x86_64 manylinux
- 推荐安装方式：`pip install <wheel URL>`

## 本地构建 wheel

本仓库使用 `hatchling` 作为构建后端，直接打包 Python 源码与预编译的 Rust 扩展：

```bash
uv build
```

构建出的 wheel 已包含 `examples/`、`README.md`、`RELEASE.md`，安装后可在 `site-packages/mfac_toolkit/` 下找到。

构建完成后，wheel 文件位于 `dist/` 目录下。可通过以下命令查看最新生成的 wheel 文件名：

```bash
ls -t dist/*.whl | head -n 1
```

## GitHub Release 发布步骤

1. 确认所有测试通过：`pytest`、`ruff check .`、`mypy mfac_toolkit`。
2. 确认 `mfac_toolkit/_mfac_core*.so` 为最新编译版本（在私有 `mfac_core` 仓库中构建）。
3. 创建并推送 tag：

   ```bash
   git tag v1.0.0
   git push origin v1.0.0
   ```

4. 在 GitHub 上进入仓库的 **Releases** 页面，点击 **Draft a new release**。
5. 选择 tag `v1.0.0`，填写发布标题与说明。
6. 上传 `dist/` 中的 wheel 文件作为 release asset。
7. 用户通过以下命令安装：

   ```bash
   pip install https://github.com/Zhaojq2003/mfac_toolkit/releases/download/v1.0.0/mfac_toolkit-1.0.0-cp312-cp312-manylinux_2_34_x86_64.whl
   ```

## PyPI 发布步骤（可选）

若选择发布到 PyPI：

```bash
uv publish dist/*.whl
```

建议仅上传 wheel，不上传 sdist，以避免源代码随 sdist 公开分发。
