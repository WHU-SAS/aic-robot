# AIC 机械臂参考资料

这个目录保留机械臂底层接口、模型、硬件联调记录与 UART 回环示例，供团队开发时核对。现有抓取程序位于 [car-arm](../car-arm/README.md)；本目录不是完整运动控制项目。全仓进度见 [状态表](../docs/status.md)。

## 当前硬件与联调入口

**舵机：ST-3215-C018；ID：1～6；波特率：1,000,000 bps。** 控制板为 CanMV-K230-LP4 V3.0，实际固件 v1.8。

**2026-10-19 实测：K230 UART1 经 URT-2 读取六个舵机全部通过，错误状态均为 0x00；测试只发送 Ping 和读取请求。**

当前接线与 UART1 物理脚号见 [硬件说明](../docs/hardware/connections.md)，完整测试、重试与更正见 [历史联调记录](../docs/history/2026-10-19-hardware-bringup.md)。测试脚本为 [UART1 回环](examples/uart1_loopback.py) 和 [六舵机只读检测](examples/uart1_servo_read.py)。

## 内容

| 路径 | 说明 |
| --- | --- |
| `hardware/FTServo_Python-main/scservo_sdk/` | 飞特舵机 Python SDK 的本地快照，包含 HLS 等协议实现 |
| `hardware/FTServo_Python-main/LICENSE` | 随 SDK 保留的 MIT 许可证与版权声明 |
| `robot_description/genkiarm.urdf` | 原参考包保留的机械臂关节与模型文件，独立于舵机 SDK |

SDK 的上游项目是 [ftservo/FTServo_Python](https://github.com/ftservo/FTServo_Python)。此处保留的是已有本地文件，未在本次整理中更新上游版本或验证与当前舵机的兼容性。使用 Python 串口接口时需在实际环境中准备 `pyserial` 等依赖，并先确认舵机型号、协议、波特率、ID、零位、方向和关节范围。

URDF 中有 6 个旋转关节，最后一关节的限制范围与其他关节不同。其 `meshes/*.stl` 外观网格没有包含在此目录中，因此不能直接用它显示完整外观；它也没有碰撞体和惯性参数，不能作为已验证的动力学模型。URDF 中的限制值不是实测设备参数。原参考包没有随此文件提供明确的来源和再分发许可记录；公开发布前需核对来源和使用权限。

## 飞特舵机常用网址

| 入口 | 用途 |
| --- | --- |
| [FTServo / FEETECH Debug Software](https://gitee.com/ftservo/fddebug/tree/master) | 飞特舵机调试软件项目入口 |
| [FT 管理平台](http://doc.feetech.cn/#/prodinfodownload?srcType=FTServo-emanual-ab4947a479784a71a8cb7928) | 飞特舵机产品资料与下载入口 |

这两个入口是机械臂舵机开发的重要参考。使用时按实际舵机型号选择对应工具与资料。

## 使用边界

- `examples/` 包含 UART1 回环和舵机只读检测脚本，不执行舵机运动。本目录没有录制动作、ROS 2 工程或网页控制平台。
- SDK 与 URDF 未在本次 UART1 联调中完成目标 Linux 构建、SDK 导入或机械臂运动验证；K230 外部回环已通过，范围见联调记录。既有抓取工程参见仓库 `car-arm/` 的说明。
- 若团队后续加入配置或实验数据，请勿提交密钥、本机路径、虚拟环境、日志及构建产物。
