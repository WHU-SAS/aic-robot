# FTServo PC SDK

来源：[ftservo/FTServo_Python](https://github.com/ftservo/FTServo_Python)，历史快照；许可见 [LICENSE](scservo_sdk/LICENSE)。

当前 pyserial 运行版保留应答 ID 与载荷长度校验补丁，避免错误包被当作寄存器读数。旧原版在 `third_party/reference/ftservo_upstream/`，不覆盖当前版。此 CPython 包不能直接运行于 K230 `machine.UART`。
