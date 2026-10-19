# 旧 car-arm 入口兼容目录

当前工作根目录为上一级 `aic-robot/`。源码、配置、模型、SDK 和文档已迁出本目录，见 [根 README](../README.md)。

推荐在仓库根目录使用：

```bash
./scripts/pick-blue --execute --hands-free
./scripts/arm-ready --execute
./scripts/check
```

原命令暂时可继续使用，例如在本目录运行 `./scripts/host hardware/move_ready.py --execute` 或 `./scripts/host hardware/k230_blue_pick_onboard.py --execute --hands-free`。兼容层只转发到同一套新实现，不加载旧参数或旧源码。

旧采集 / 日志不会自动导入；新输出在根目录忽略提交的 `data/`。本目录不再新增开发代码。
