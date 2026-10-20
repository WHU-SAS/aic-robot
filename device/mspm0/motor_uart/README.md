# K230 控制底盘：启停与状态回传

当前天猛星运行本固件。2026-10-20 已完成 USB/BSL 下载、重复映像 CRC 和写入范围外 CRC 核验、只读状态 20/20，以及架空轮启停测试；用户确认两轮正常转动并停止、无异常。

## 已确认的首轮运行方式

- 上电停车，新的 START 才启动；方向沿用 `motor_encoder`，A/B 尚未对应到左右轮。
- 10 kHz PWM，约0.5秒升到1600/8000（20%占空比），每次最多5秒。
- K230 每轮心跳后等待200毫秒；连续1秒无新有效心跳，天猛星停止输出。STATUS 不续期；恢复心跳不会自动启动。
- B21 只停车，不启动；30毫秒消抖，按住时拒绝 START。2026-10-20实机短按停车通过，停车后继续心跳观察2秒未重启，编码器最终稳定。
- 停车令 STBY=0、PWM=0，车轮可惯性滑行；不代表机械制动或车速立即为零。

机械臂不参与本测试；PID、准备位和机械臂运行参数不变。

## 运行

接线与原例程一致：[底盘引脚](../motor_encoder/README.md#接线)。K230 IO5 TX→PA09 RX，IO6 RX←PA08 TX，GND共地，115200/8N1。IDE停止并断开。

仓库根目录执行，可重复运行，无须重启两板：

```bash
./scripts/host tools/diagnostics/test_k230_motor_uart.py                # 20轮状态查询，不启动
./scripts/host tools/diagnostics/test_k230_motor_uart.py --stop         # 停止输出并复查
./scripts/host tools/diagnostics/test_k230_motor_uart.py --execute --wheels-lifted
./scripts/host tools/diagnostics/test_k230_motor_uart.py --execute --wheels-lifted --button
```

最后一条必须先可靠架空两轮、手离开车轮。自动序列：坏校验帧不得启动 → START及重复命令 → 缓升并读取两路编码器 → STOP → 重新启动验证5秒时限 → 重新启动后不发心跳，验证1秒断链停车 → 最终STOP。遇错尝试STOP；脚本或USB连接消失后，由天猛星定时中断执行心跳超时。

`--button`单独测试B21：先等待5秒，再启动一次；车轮转动时短按B21，要求收到`reason=2`及零输出，继续发送心跳观察2秒不得重启。没有按到、只触发5秒时限或其他异常均判失败，不自动重试。

记录在本机 `data/logs/k230_motor_uart_*/`，不提交。首次实测：5秒停车首次被轮询观察到为5.18秒，断链为1.00秒；这是采样观察时刻，非直接测量PWM边沿。两路原始计数均变化，尚未标定每圈计数、轮径或换算速度，也没有恒转速闭环。

## 协议

请求 `@M,SSSSSSSS,COMMAND,CCCC\n`，命令为 `STATUS/START/STOP/HB`。
应答 `@R,SSSSSSSS,COMMAND,result,ms,run,pwm,encoder_a,encoder_b,reason,last_seq,CCCC\n`。

序号为8位大写十六进制，CRC为4位大写十六进制。CRC-16/CCITT-FALSE（初值FFFF，多项式1021，无反射及最终异或）覆盖 `@` 之后、校验字段前最后一个逗号之前的字节。`123456789` 对应29B1。

- `result`：0成功，1重复未重做，2旧序号，3正在运行或按键按住而拒绝START。
- `reason`：0上电/运行，1STOP，2B21，3五秒时限，4心跳丢失。
- `ms` 为上电毫秒，`encoder_a/b` 为有符号原始累计计数（32位回绕）；复位归零。
- 除STATUS外，接受的新命令推进 `last_seq`，按32位序号前后关系判断。新客户端先STATUS读取该值再发送新序号；单个控制端使用，不能并行启动多个客户端。
- 重发START不重新启动、不延长时限；运行中新的START返回忙。重复HB不续期，旧STOP仍停车。
- 帧过长、命令错误、校验错误不执行；任何位置的 `@` 重新同步，兼容此前冷启动前导噪声。

定时中断每10毫秒处理PWM与两项超时；串口解析、CRC和应答在前台，STOP在应答前实际撤去输出。无主动推送，以命令/心跳应答返回状态。

## 构建与下载

```bash
./scripts/mspm0-build motor_uart
./scripts/mspm0-tools python -m unittest discover -s device/mspm0/tests -v
./scripts/mspm0-tools python tools/diagnostics/mspm0_bsl.py \
  --hex device/mspm0/motor_uart/build/gcc/motor_uart.hex --execute
```

下载按键和CRC复核见[板端联调](../../../docs/guides/k230-and-mspm0.md#当前下载方式主控-usb--bsl)。本工程目前提供Linux GCC/BSL入口，无单独Keil工程；复用 `motor_encoder/ti_msp_dl_config.c/.h`，原独立按键例程保持可用。UART1使用电机配置下的40MHz BUSCLK，波特率分频21/45，不能套用32MHz独立UART例程的17/23。
