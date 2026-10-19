# FTServo PC 运行 SDK

来源：[ftservo/FTServo_Python](https://github.com/ftservo/FTServo_Python)，本地历史快照，未宣称某个未经确认的上游 tag。许可见 [scservo_sdk/LICENSE](scservo_sdk/LICENSE)。

本次从原 `car-arm/ros2_ws/src/arm_pkg/arm_pkg/scservo_sdk/` 逐字节迁移，PC 通过 pyserial 使用。与隔离的旧原版相比，`protocol_packet_handler.py` 保留应答 ID、期望载荷长度校验；不让合法校验和但错误 ID / 长度的包成为寄存器读数。其余差异含换行 / 空白，未更新 SDK 版本。

K230 `machine.UART` 只读诊断继续独立，不能把此 CPython / pyserial 包直接用于板端。
