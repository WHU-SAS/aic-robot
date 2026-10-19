# 目录迁移记录（2026-10-20）

迁移前提交：`be5f48f82044a2143352449938e046e8b2e9fdd1`。逐文件对应见 [清单](history/reorganization_manifest.json)，技术依据见 [原审查](history/2026-10-20-project-review.md)。

## 结果

- PC 程序拆为 host/apps、arm、vision、transport；采集 / 标定 / 分析归入 tools。
- 板端脚本与固件归入 device；配置、模型、完整SDK和硬件资料分别归位。
- FTServo 使用带应答 ID / 载荷校验的运行版。旧 SDK、URDF 与模板隔离在 third_party/reference，不加入运行或构建路径。
- 根命令统一为 scripts/pick-blue、arm-ready、check；旧 car-arm 命令只转发。
- 数据与控制锁写入忽略的 data；运行内外参在 configs/vision/runtime。
- 复审补齐配置说明路径和工具入口，合并7份重复报告。原始提交与文档保留在 Git 历史。

PID、速度、控制阶段、准备位和标定数值未改变。配置里的历史来源字段只用于追溯，不作为运行文件路径。

## 验证

三组合成编码的 FK 和完整四阶段 IK 与迁移前一致。完整TI SDK1438文件、小车硬件资料17文件及全部模型保留。默认离线检查、52项测试、干净检出与文档链接检查通过。

两个 MSPM0 工程的源码、include、库与scatter路径检查通过，主机电机逻辑测试通过。迁移后尚未在 Windows Keil 实际重编译或烧录；本轮未重新真机验收，硬件进度见 [状态表](status.md)。
