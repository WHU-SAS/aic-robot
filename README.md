# AIC 小车与机械臂

WHU-SAS 团队的 AIC 开发代码与硬件参考资料。

| 目录 | 内容与入口 |
| --- | --- |
| [car/demo/01_motor_encoder/](car/demo/01_motor_encoder/) | MSPM0G3507 双电机与编码器实验，参见 [使用说明](car/demo/01_motor_encoder/使用说明.md) |
| [car/demo/02_uart1_link/](car/demo/02_uart1_link/) | K230 UART2 ↔ MSPM0 UART1 通信测试，参见 [接线与测试说明](car/demo/02_uart1_link/README.md) |
| [car/硬件信息/](car/硬件信息/) | 当前硬件照片、采购记录和 [硬件说明](car/硬件信息/现有硬件与功能.txt) |
| [arm/](arm/) | 飞特舵机 Python SDK 与机械臂 URDF，参见 [机械臂说明](arm/README.md) |
| [car-arm/](car-arm/) | CanMV K230定位蓝色20mm方块与六轴机械臂抓取演示，参见 [运行说明](car-arm/README.md) |

## 小车工程

保留 `car/demo/01_motor_encoder/source/` 内的 TI SDK 副本，保持原工程内部目录结构。构建命令和接线以小车使用说明为准；构建输出不纳入 Git，克隆后需要本机重新编译。

完整 WHEELTEC 厂家资料包和 `vendor_reference/` 属于本机参考资料，未纳入此仓库。小车“硬件信息”中的全部 17 个文件已纳入版本管理。

## 机械臂资料

SDK 和 URDF 分别放在 `arm/hardware/` 和 `arm/robot_description/`。SDK 保留原 MIT 许可证；URDF 的网格、碰撞体、惯性参数及来源限制见机械臂说明。

机械臂舵机为 **ST-3215-C018，ID 1～6，1,000,000 bps**。K230 V3.0/v1.8、UART1 引脚、URT-2 接线及回环结果见 [硬件与联调记录](arm/docs/硬件与联调记录.md)。

## K230与机械臂抓取

`car-arm/`为可独立运行的K230识别→PC位姿/FK/IK→六轴同步抓取模块。当前已由用户现场接受：30°张开、15mm下探、目标径向额外伸展10mm，夹爪连续0.5秒静止后抬升。原始照片、实验数据、运行日志和已安装依赖不提交；当前运行标定与模型保留在模块内。

从仓库根目录进入模块运行，完整环境和串口权限步骤见模块README：

```bash
cd car-arm
./scripts/host hardware/k230_blue_pick_onboard.py --execute --hands-free
```

### 小车行驶时保持机械臂准备位

**小车行驶过程中，机械臂必须保持准备位和扭矩，以维持整车平衡。** 行驶前在`car-arm/`目录执行并确认到位：

```bash
./scripts/host hardware/move_ready.py --execute
```

准备位为六轴`0°、−80°、80°、80°、0°、30°`，与中立位不同。当前未自动与小车启动联动；抓取等改变姿态的操作在小车停稳后进行。准备位包含张开夹爪目标，持物行驶策略另外确定。详见[回准备位说明](car-arm/docs/准备位脚本.md)。

## 团队协作

每位成员使用自己的 GitHub 账号克隆仓库。修改前同步 `main`，在自己的分支上开发，通过 Pull Request 合并。不要提交密钥、本机环境、日志和构建产物。

本仓库按私有团队开发资料管理；公开发布前核对第三方资料的授权。各子目录中的许可证适用于对应文件，不对整个仓库统一声明开源许可证。

合并保留了原小车与机械臂仓库的提交历史。本次仅做仓库整合，未执行固件构建或真机验证。
