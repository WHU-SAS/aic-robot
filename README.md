# AIC 小车与机械臂

WHU-SAS 团队的 AIC 开发代码与硬件参考资料。

| 目录 | 内容与入口 |
| --- | --- |
| [car/demo/01_motor_encoder/](car/demo/01_motor_encoder/) | MSPM0G3507 双电机与编码器实验，参见 [使用说明](car/demo/01_motor_encoder/使用说明.md) |
| [car/硬件信息/](car/硬件信息/) | 当前硬件照片、采购记录和 [硬件说明](car/硬件信息/现有硬件与功能.txt) |
| [arm/](arm/) | 飞特舵机 Python SDK 与机械臂 URDF，参见 [机械臂说明](arm/README.md) |

## 小车工程

保留 `car/demo/01_motor_encoder/source/` 内的 TI SDK 副本，保持原工程内部目录结构。构建命令和接线以小车使用说明为准；构建输出不纳入 Git，克隆后需要本机重新编译。

完整 WHEELTEC 厂家资料包和 `vendor_reference/` 属于本机参考资料，未纳入此仓库。小车“硬件信息”中的全部 17 个文件已纳入版本管理。

## 机械臂资料

SDK 和 URDF 分别放在 `arm/hardware/` 和 `arm/robot_description/`。SDK 保留原 MIT 许可证；URDF 的网格、碰撞体、惯性参数及来源限制见机械臂说明。

## 团队协作

每位成员使用自己的 GitHub 账号克隆仓库。修改前同步 `main`，在自己的分支上开发，通过 Pull Request 合并。不要提交密钥、本机环境、日志和构建产物。

本仓库按私有团队开发资料管理；公开发布前核对第三方资料的授权。各子目录中的许可证适用于对应文件，不对整个仓库统一声明开源许可证。

合并保留了原小车与机械臂仓库的提交历史。本次仅做仓库整合，未执行固件构建或真机验证。
