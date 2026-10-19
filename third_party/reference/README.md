# 隔离的旧参考文件

这些内容原样归档，保持来源可追溯；**不加入运行导入、include 或构建路径**。

| 路径 | 原用途与保留原因 |
| --- | --- |
| `ftservo_upstream/` | 原 `arm/hardware/FTServo_Python-main/` SDK 和 MIT 许可证；与当前运行补丁版区分，不能覆盖它 |
| `genkiarm_original.urdf` | 原始参考 URDF；部分角度为1.57，和当前运行的1.57079632679不同；无完整网格，不作为默认模型 |
| `motor_template/` | 原电机目录未参与构建的 empty.c/.h、Hardware、平铺ti、旧Keil工具与打包脚本；后续Windows重编译前保留，不混用到完整SDK |

已检查两个 build.py、两个 Keil 工程及当前应用 include；当前构建输入不引用这些模板。平铺驱动有13个同名异内容文件，因此按隔离归档处理，不盲目去重。`empty.syscfg` 单独保留于 motor_encoder/reference，尚未工具重新生成验证。

逐文件原位置见 [迁移清单](../../docs/history/reorganization_manifest.json)。本目录没有旧 PID 实验或原始采集数据。
