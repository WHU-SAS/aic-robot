# 硬件联调摘要（2026-10-19）

设备：CanMV-K230-LP4 V3.0（固件v1.8、MicroPython1.21）、URT-2、ST-3215-C018六舵机、MSPM0G3507。接线见 [硬件说明](../hardware/connections.md)。

## 测试结果

| 项目 | 结果 |
| --- | --- |
| K230 UART1 回环，GPIO3/4，115200/8N1 | 外部短接后10/10通过 |
| K230 UART1→URT-2→舵机，1Mbps | ID1～6 Ping及身份/状态读取6/6通过，型号编码777，错误均0x00；仅只读测试 |
| MSPM0 UART1测试固件 | 编译及HEX检查通过，Main Flash1672字节；尚未烧录成功 |
| K230 UART2↔MSPM0 UART1 | 接线已确认，双板通信尚未验证 |

对应脚本：[回环](../../device/k230/diagnostics/uart1_loopback.py)、[舵机只读](../../device/k230/diagnostics/uart1_servo_read.py)、[UART工程](../../device/mspm0/uart_link/README.md)。

## SWD故障结论

- Keil报 `E310: cannot read DP register 0x00`；独立CMSIS-DAP检测在100kHz、10kHz下均为 `NO_ACK`。
- 普通复位及BSL+RST尝试未恢复连接；120秒内965次读取均无应答，未独立确认进入BSL。未擦除、解锁或写入芯片。
- 2026-09-29日志曾读到DP ID `0x6BA02477`；本次故障原因仍未确定。
- 用户确认SWD信号接线正确；此前根据照片推断误接5V/TX/RX的判断已撤回。板上3V3/5V尚无实测值，换板尚未确认。

## 接线依据

- [URT-2官方接线图](https://raw.githubusercontent.com/ftservo/ftservo-wiki/main/docs/assets/images/wiring/urt-2-ttl-wiring.png)
- [K230 V3.0硬件设计](https://download.kendryte.com/developer/k230/HDK/CanMV-K230%E5%B7%A5%E7%A8%8B%E8%AE%BE%E8%AE%A1%E6%96%87%E4%BB%B6/ProDocument_CanMV-K230-LP4-V3.0-20240509.epro)
- [CanMV v1.8 UART1示例](https://github.com/kendryte/canmv_k230/blob/v1.8/resources/examples/03-Machine/uart1.py)
