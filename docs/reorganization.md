# 项目整理结果

日期：2026-10-20。第一批文档已合入 main；剩余目录迁移在同一轮完成并按用户要求直接上传主分支。起点 `be5f48f82044a2143352449938e046e8b2e9fdd1`，逐文件来源见 [迁移清单](history/reorganization_manifest.json)，最初发现见 [原审查](history/2026-10-20-project-review.md)。

## 当前结构

```text
aic-robot/
├─ device/
│  ├─ mspm0/{motor_encoder,uart_link,platform}/
│  └─ k230/{vision,diagnostics}/
├─ host/{apps,arm,vision,transport,tests}/
├─ tools/{capture,calibration,analysis}/
├─ configs/{arm,vision/runtime}/
├─ models/genkiarm/
├─ third_party/{ti_mspm0_sdk,ftservo_python,reference}/
├─ hardware/car/
├─ docs/{hardware,guides,history}/
├─ scripts/
└─ car-arm/                    # 旧命令薄兼容层
```

`hardware/arm/` 没有另建空目录；当前机械臂接线资料在 docs/hardware，旧模型与 SDK 的归档在 third_party/reference。本机 data/、依赖环境不纳入 Git。

## 已完成

- PC 应用、运动学、视觉、通信分层；采集 / 标定 / 分析工具单独归位。
- 从 analyze_grasp_batch 抽出三个运行函数至 host/vision/face_geometry.py，数值不变；运行代码不再借用分析脚本。
- K230 板端识别 / 诊断独立于 PC，源码拼接上传适配新路径；板端源码保持原字节。
- 配置独立至 configs，运行内外参冻结文件与 provenance 字节保持；pick JSON 仅修改内外参路径。
- 当前 URDF 和全部 STL 原样迁移；CCC / CCC2 名称及引用保留。
- 运行 FTServo SDK 统一从 third_party/ftservo_python 导入，保留 ID / 载荷校验补丁和 MIT 许可证。
- 完整 TI SDK 1438 文件原样迁移；共享启动 / scatter 独立，两个 build.py 与两个 Keil 工程一起适配。
- 小车全部17个硬件资料原样保留。旧 SDK、URDF、未参与构建的模板 / 驱动 / 工具隔离至 reference，不放入运行或构建路径；此次没有丢弃原受控资料。
- 根命令 arm-ready / pick-blue / check 就位；旧 car-arm 命令转发至同一实现，没有第二套参数或控制代码。
- 数据输出与控制锁归入忽略的 data/；无旧 PID 试验导入、无原始采集 / 日志 / 已安装依赖上传。

## 控制与数值不变

未修改 PID、积分限幅、最小启动力、死区、速度 / 加速度、同步发送策略、准备位、夹爪对齐定义、30°张开、15mm模型下探、径向+10mm、额外闭合20计数、0.5秒夹爪静止条件或终态扭矩策略。没有执行真机动作。

配置中闭合量 / 观察时间与代码内仍存在相同值的双重定义，此次保留实际行为。后续收敛配置来源应单独处理，不混入结构迁移。

## 验证

- 默认离线检查通过，既有43项测试及新增6项迁移回归通过。
- 迁移前后三个合成编码姿态的 FK 报告、完整四阶段 IK 计划完全一致；合成数值基准纳入 host/tests/fixtures，不是实测采集或旧 PID 试验数据。
- 当前模型、内外参、SDK、板端脚本、硬件资料哈希核对一致；控制 JSON 值不变，pick 仅路径更新。
- 新旧准备位入口离线输出一致，host / vision 两套环境与新入口检查通过。
- 两份 Keil 工程的源码 / include / 静态库 / scatter 路径存在；两个 build.py --check-paths 通过。Linux GCC 主机电机控制逻辑测试通过。
- 干净暂存快照的离线检查、测试与本地文档链接检查结果见 [本轮记录](history/2026-10-20-layout-migration.md)。

**本机没有 Windows Keil：迁移后的两个固件未实际重编译或烧录。** build.py --check-paths 和 XML 检查不等于编译通过。新电脑从零联网安装、真机抓取与底盘联调也未在本轮重做；原硬件进度不变，见 [状态表](status.md)。

## 后续工作入口

从根目录 README / docs/status 开始。用户主导控制设计；后续完成验证后直接合入并上传 main，不再要求用户手动合并。本轮不实现指定位置放置、K230 原生抓取或小车自动联动。
