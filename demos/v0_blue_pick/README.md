# 第0版：蓝色方块抓取可行性演示

已在2026-10-13现场跑通。K230提取特征，PC完成定位、FK/IK和机械臂控制。

```bash
./scripts/demo-v0-pick --detect-only
./scripts/demo-v0-pick --execute --hands-free
```

原`./scripts/pick-blue`及`host/apps/k230_blue_pick*.py`入口保留兼容。

## 文件与基线

- `run.py`：原四阶段演示流程。
- `pick.json`：演示专属参数，包含前伸10mm、30°开口、15mm下探、闭合停稳后抬升。
- `baseline.json`：冻结版本与识别核心哈希。
- `vision-v0-20261021`标签：整理前完整源码；共享算法以当前模块为准，需要精确复现时使用标签版本。
- `configs/arm/k230_pick_config.json`保留为旧路径指引，参数实际位于本目录。

当前PID、速度/加速度仍读取`configs/arm/`已保存基线，保持原动作策略。结束保持抬升和扭矩。

蓝色识别核心继续在`device/k230/vision/`维护；视觉配置见`configs/vision/blue_cube.json`。内外参是第0版临时标定参考，前伸补偿属于本演示的动作配置。

正式版已实现K230独立语音抓取，整车任务调度尚未接入。开发说明见[视觉模块](../../docs/guides/vision.md)。
