# MSPM0 双电机与编码器示例

面向立创天猛星 MSPM0G3507、TB6612 驱动板、两台 MG513XP28_12V 电机。入口为 `main.c`，启停逻辑在 `demo_control.c/.h`，板级配置为 `ti_msp_dl_config.c/.h`。

## 使用

先读 [接线、编译、下载和分步验收](使用说明.md)。在本目录、Windows Keil Arm Compiler 6 环境中编译：

```powershell
python build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
```

Keil 工程为 [motor_demo.uvprojx](keil/motor_demo.uvprojx)。构建输出 `build/motor_demo.hex` / `.axf` 不纳入 Git，克隆后本机生成。

上电停止；松开后按 B21 启动，约 0.5 秒缓升到 20% PWM，再按键或运行 5 秒停车。这是车轮悬空的入门实验，不是恒转速闭环控制。编码器输出原始计数，未套用未经确认的减速比 / 每圈计数。

## 状态与依赖

2026-09-29 已有编译及主机逻辑测试记录；实物 PWM、电机和编码器验收未记录完成。当前全仓进度见 [状态表](../../../docs/status.md)。

`source/` 中完整 TI SDK 继续保留；相邻 UART 示例还复用本目录的 SDK、启动汇编和 scatter 文件。`empty.c/.h`、`Hardware/` 和平铺 `ti/` 是待核查参考内容，不参与当前应用构建；本批没有删除。`empty.syscfg` 尚未通过工具重新生成验证，当前构建使用已保存的 `ti_msp_dl_config.c/.h`。

项目将按 [迁移清单](../../../docs/reorganization.md) 后续拆分共享依赖。本次只替换原空工程模板说明，未修改程序或工程文件。
