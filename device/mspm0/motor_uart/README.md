# K230 控制底盘：启停与状态回传

当前天猛星运行本固件。2026-10-20 已完成 USB/BSL 下载、重复映像 CRC 和写入范围外 CRC 核验、只读状态 20/20，以及架空轮启停测试；用户确认两轮正常转动并停止、无异常。

## 已确认的首轮运行方式

- 上电停车，新的 START 才启动；方向沿用 `motor_encoder`，A=左轮、B=右轮（10月20日用户手转左轮、只读采样确认）；用户确认当前运行方向为车头方向，前进计数A正、B负。
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

记录在本机 `data/logs/k230_motor_uart_*/`，不提交。首次实测：5秒停车首次被轮询观察到为5.18秒，断链为1.00秒；这是采样观察时刻，非直接测量PWM边沿。两路原始计数均变化。尚未实测每圈计数或距离比例，也没有恒转速闭环；后续先按下述资料标称参数换算。

## 标称每圈计数与短前进

参数集中于 [drive_nominal.json](../../../configs/car/drive_nominal.json)。本机照片标签是MG513XP28_12V；28:1按P28标称型号采用，并由[同型号系列的原作者硬件项目](https://oshwhub.com/ylc00/simple-pid)交叉核对。该项目是霍尔版，不能用其13线数替代本机编码器版本。

[轮趣电机开发手册表3-1（转载副本）](https://www.scribd.com/document/1000178757/Wheeltec-直流电机开发手册-2025-10-20)列出GMR500ppr带塑料后盖，霍尔13ppr为裸露。结合本机后盖照片及已有计数规模，本项目暂按GMR500ppr计算；这是版本推定，尚无订单型号或整圈测量确认，不能表述为实测500线。若版本判断改变，必须重新计算比例。

| 参数 | 初始值与依据 |
| --- | --- |
| 轮胎直径 | 65 mm，采购清单标称值 |
| 减速比 | 28:1，MG513XP28型号标称 |
| 编码器 | 暂按GMR500ppr，电机转子每圈单相周期数 |
| 解码倍频 | 2，程序对AB两相各采上升沿 |
| 轮子每圈计数 | 500 × 28 × 2 = **28000** |
| 轮子周长 | π × 65 = **204.2035 mm** |
| 每计数距离 | **0.00729298 mm**，对应每100 mm约13711.81计数 |

霍尔13线版在相同减速比/2倍频下是728计数/圈，不能与28000混用；4倍频时GMR才是56000。本轮手转没有数圈，不用于推算减速比或PPR。当前软件中断计数存在丢边沿可能，标称比例也不等于实测里程精度。

首次地面测试只做**约1秒前进后STOP**，输出仍为原20% PWM、0.5秒缓升；5秒运行时限和1秒断链停车仍在。不按估算距离闭环停车，不自动重试，不自动调PID；结束后再读取2秒计数以包含滑行。PC根据标称参数输出两轮位移，明确标注未实测标定。2026-10-20已在机械臂保持准备位时完成首次落地实测，用户反馈无异常：驱动约1040 ms，含滑行左轮19159计数、右轮17678计数，标称换算平均134.3 mm；约11 mm轮程差不等于已测得的车身航向偏差。停车后PWM为0，编码器稳定。持续带载及精确定距/转向尚未验证。

在地面路径清空、线缆留有余量、机械臂已回准备位并保持扭矩、现场有人观察后执行：

```bash
./scripts/host tools/diagnostics/test_k230_motor_uart.py --execute --forward-once --on-ground
```

架空测试同一动作则将 `--on-ground` 换成 `--wheels-lifted`。两个标志不可同时使用。约1秒由K230脚本计时并发STOP，串口往返有延迟；不是固件新增长度1秒的硬时限，也不是定距命令。固件无须重烧，机械臂准备位另用既有 `./scripts/arm-ready --execute`，本脚本不控制机械臂。

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
