# PC识图兼容入口

当前默认入口为[板端识别抓取](K230_ONBOARD_PICK.md)。保留PC端OpenCV识图方式：

```bash
./scripts/host host/apps/k230_blue_pick.py --execute --hands-free
```

该入口从K230导出1280×960照片，PC提取物理面并计算位姿；随后与板端入口共用`k230_blue_pick.py`执行代码、`k230_pick_config.json`、当前标定快照及IK。

当前共用30°开口/2389、15mm下探、径向伸展10mm、接触后额外闭合20计数、连续0.5秒静止后抬升。结束保持抬升和扭矩。板端方式通常省去正常成功时JPEG传输；以现场板端入口为默认版本。

默认抓取不调用基础手动任务入口的逐阶段确认模式；`host/apps/run_task.py`继续用于其自身手动任务，两种入口的推进方式分别保留。

历史调试过程、旧补偿与旧高度记录留在本地`docs/history/pre_git_20261013/`，不作为当前设置。
