# 双电机与编码器例程

MSPM0G3507 + TB6612 + 两台 MG513XP28_12V。当前为开环 PWM，编码器输出原始计数。

## 运行行为

上电停止；松开 B21 后按下，0.5 秒内升到 20% PWM（10 kHz），再按一次或运行 5 秒自动停车。持续按住不反复触发。停车拉低 STBY，车轮可能惯性转动。

2026-09-30 的[硬件记录](../../../hardware/car/现有硬件与功能.txt)记载按键启停、PWM、5 秒停车、编码器计数和前进测试已完成。2026-10-20 Linux GCC 固件实机验收通过：上电停止、两轮手转正反计数、B21 启动、约 0.5 秒升至 PWM 1600、约 5 秒超时停车（`stop_reason=2`）。用户确认两轮运行正常；停车后反向计数为手动转轮。当前主控已运行另一个 [K230 UART 电机联调固件](../motor_uart/README.md)，本目录保留独立按键例程。轮径先按资料标称65 mm；每圈计数暂按GMR500ppr×28减速比×2倍频=28000，版本推定与资料来源见 [参数说明](../motor_uart/README.md#标称每圈计数与短前进)，轮距未确定，也未实现恒转速闭环。

## 接线

先断电。主控由 USB 供电，电池给驱动板供电，两板共地；不将驱动板 5V/3V3 输出接到已由 USB 供电的主控。

| TB6612 板信号 | MSPM0 引脚 | 用途 |
| --- | --- | --- |
| GND | GND | 共地 |
| PWMA / PWMB | PB2 / PB3 | A / B 路 PWM |
| AIN1 / AIN2 | PA14 / PA13 | A 路方向 |
| BIN1 / BIN2 | PA16 / PA17 | B 路方向 |
| STBY | PA12 | 低电平停车，不能再同时接电源 |
| E1A / E1B | PA25 / PA26 | A 路编码器 |
| E2A / E2B | PB20 / PB24 | B 路编码器 |

电机使用配套六芯线。首次接编码器前，确认 E1/E2 高电平不超过 3.3 V；AO/BO 为电机功率输出，不能接 MCU。按丝印接线。本车已于2026-10-20只读确认A=左轮、B=右轮；当前测试前进方向的原始计数A正、B负。

## 编译与下载

Linux 在仓库根目录运行 `./scripts/mspm0-build motor_encoder`，生成 `build/gcc/motor_demo.elf` / `.hex` / `.bin`。

Windows Keil 编译：

```powershell
python device/mspm0/motor_encoder/build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
```

生成本目录 `build/motor_demo.hex` / `.axf`。Keil 和 USB / BSL 下载共用 [板端联调步骤](../../../docs/guides/k230-and-mspm0.md)。Linux SWD、UART 固件烧录与 GDB 调试已验证；电机固件的 Linux 烧录、回读及悬空车轮测试也已通过。其他项目进度见 [项目首页](../../../README.md#当前情况)。

入口 `main.c`，启停逻辑 `demo_control.c/.h`，引脚配置 `ti_msp_dl_config.c/.h`。`reference/empty.syscfg` 尚未重新生成验证；Keil 不自动运行 SysConfig，以当前 C 配置为准。重新生成时复核按键上拉、STBY、定时器启动及 UART 回环。旧模板隔离在 `third_party/reference/motor_template/`，不参与构建。

## 分步验收

1. **只接主控 USB**：串口 115200/8N1，每 100 ms 一行；初始 `run=0,pwm=0`，B21 启动后 PWM 升至 1600，再按或 5 秒后归零。
2. **车轮悬空，检查编码器**：断电接线后再上电，保持 `run=0`；手转一轮，对应计数变化，反转符号相反，另一通道基本不动。
3. **车轮悬空，检查电机**：按 B21，两轮缓升转动，再按或 5 秒后停驱动。运行中不要暂停调试器，暂停 CPU 可能保留 PWM 并使软件超时失效。

20% PWM 可能不足以起转；先停车检查供电，避免堵转。需要改驱动量时修改 `DEMO_PWM_TARGET`（PWM 周期 8000）并重新编译；方向不符合时修改 `main.c` 的 `MOTOR_A_REVERSE` / `MOTOR_B_REVERSE`。

## 串口字段

```text
ms,run,pwm,a_100ms,b_100ms,a_total,b_total,stop_reason
```

- `ms`：启动后毫秒；`run`：0 停止 / 1 运行；`pwm`：0～1600，对应 0～20%。
- `a_100ms/b_100ms`：最近 100 ms 有符号计数；`a_total/b_total`：开机累计，停车不清零，RST 清零。
- `stop_reason`：0 上电或运行、1 按键停车、2 超时停车。

两相只采上升沿（2 倍频），积压中断时丢弃歧义样本，不保证高速无丢计数。计数未经左右轮符号归一化，也未换算 RPM；不要直接套用厂家例程的 13 线、30:1。

参考：[立创 B21](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/ccs-beginner/key.html)、[硬件原始资料](../../../hardware/car)。厂家原工程来自 WHEELTEC C07A TB6612 示例，保留 TI 与厂家文件版权。
