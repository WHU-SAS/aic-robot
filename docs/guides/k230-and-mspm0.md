# CanMV 与 Windows Keil：板端联调

本页为现有脚本导航。接线依据 [硬件说明](../hardware/connections.md)，结果依据 [验证状态](../status.md)。Linux 的 PC 抓取命令不能直接当作 CanMV 脚本运行。

## CanMV K230 UART1 回环

在 CanMV IDE K230 中连接开发板，打开并运行 [uart1_loopback.py](../../arm/examples/uart1_loopback.py)。GPIO3 与 GPIO4 外部短接，断开 URT-2 等外设；参数为 115200/8N1。已记录外部回环 10/10 通过，完成后拆除短接线。

## CanMV K230 直连舵机，只读

按硬件说明连接 URT-2，电平 3V3、URT-2 USB 拔除、舵机独立供电。IDE 打开并运行 [uart1_servo_read.py](../../arm/examples/uart1_servo_read.py)。脚本只对 ID 1～6 发送 Ping / READ，1 Mbps，检查应答长度、ID、校验和与错误状态。

已记录 `RESULT: 6 / 6 servos read`、`ALL_SERVOS_READ_OK` 且错误均 0x00。此结果不代表原生抓取通过。脚本不保存成开机自动运行的 main.py，也不调用 Linux pyserial SDK。

## Windows：MSPM0 工程

需要 Keil Arm Compiler 6、匹配 MSPM0G3507 的 TI 设备包及相应调试器。默认工具链目录为 `D:/Keil_v5/ARM/ARMCLANG/bin`，可通过 `--toolchain` 覆盖。

从仓库根目录进入目标示例后编译：

```powershell
# 电机 / 编码器示例
cd car/demo/01_motor_encoder
python build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
```

或另开终端，从仓库根目录执行：

```powershell
# UART1 独立测试固件
cd car/demo/02_uart1_link
python build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
```

目前两个示例复用电机目录内的完整 TI SDK、启动与链接文件。第二批 / 第三批迁移完成前不要手动删除这些依赖。`build/` 是本机输出，不随 Git 提供；克隆后需本机编译。

电机接线、按键逻辑、下载和分步验收见 [使用说明](../../car/demo/01_motor_encoder/使用说明.md)。UART 工程的 [Keil 文件与下载步骤](../../car/demo/02_uart1_link/README.md) 覆盖 fireDAP / SWD 设置；当前 E310 / NO_ACK 尚未解决，不能跳过烧录成功确认直接宣称链路通过。

MSPM0 UART 固件下载并运行后，CanMV IDE 执行 [k230_uart2_test.py](../../car/demo/02_uart1_link/k230_uart2_test.py)。验收要求 20/20 带序号、令牌的 `MSPM0_ACK`，原样回显不算成功。**这是待完成验收，不是当前实测结果。**

Windows 本机 IDE 路径、COM 号与探针选项见历史记录，仅供追溯。本轮文档整理未在 Linux 上重新编译 Windows 工程。
