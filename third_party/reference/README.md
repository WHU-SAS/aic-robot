# 隔离参考

以下文件不加入运行、include 或构建路径。

| 路径 | 内容 |
| --- | --- |
| `ftservo_upstream/` | 未加当前应答校验补丁的旧 SDK，含 MIT 许可 |
| `genkiarm_original.urdf` | 原参考模型，角度 1.57；运行模型使用 1.57079632679，不能互换 |
| `motor_template/` | 未参与构建的 empty.c/.h、Hardware、平铺 TI 驱动、旧 Keil 工具和打包脚本 |

平铺驱动有 13 个同名异内容文件，保留供对照；实际 MSPM0 工程使用 `third_party/ti_mspm0_sdk/` 与 `device/mspm0/platform/`。SysConfig 待复核配置单独保存在 `device/mspm0/motor_encoder/reference/empty.syscfg`。
