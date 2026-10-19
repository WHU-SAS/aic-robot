# 项目整理计划与迁移清单

更新：2026-10-20。用户已同意先完成第一批文档整理。**本页目标目录是后续布局，不是当前可运行路径；当前命令继续使用 `car-arm/`。**

整理前基准为 `1a2d23cc28c10f4a7348fe1f39ba5e371238a503`，共 1672 个受控文件。原报告按更早提交计数，见 [历史审查](history/2026-10-20-project-review.md)。整理分支保留原提交历史，回退通过 Git 完成，不复制一套旧项目。

## 目标职责

```text
aic-robot/
├─ README.md
├─ device/
│  ├─ mspm0/{motor_encoder,uart_link,platform}/
│  └─ k230/{vision,diagnostics}/
├─ host/{apps,arm,vision,transport,tests}/
├─ tools/{capture,calibration,analysis}/
├─ configs/{arm,vision}/
├─ models/genkiarm/
├─ third_party/{ti_mspm0_sdk,ftservo_python}/
├─ hardware/{car,arm}/
├─ docs/{hardware,guides,history}/
│  ├─ status.md
│  └─ reorganization.md
└─ scripts/
```

- `device/` 表示实际在板上执行的程序，包括 CanMV Python，不暗示全部是编译固件。
- `host/` 放 PC 运行模块；`tools/` 放采集、标定、分析工具。工具可依赖运行库，运行库不依赖历史分析脚本。
- `configs/` 在根目录独立保存当前配置，便于 PC 与后续 K230 原生方案共享来源；不因此自动实现板端控制。
- `hardware/` 只放硬件资料；模型、第三方库、代码各自归位。
- 环境入口继续分 host / vision；不在此次整理中统一 NumPy 环境。

## 分批范围与状态

| 批次 | 内容 | 状态 / 验收 |
| --- | --- | --- |
| 第一批 | 根导航、验证矩阵、当前接线、平台指南、历史索引与迁移清单 | 本批完成文档整理；源码和配置未迁移，记录见 [本批检查](history/2026-10-20-docs-batch1.md) |
| 第二批 | car-arm 运行库、工具、配置、模型、PC FTServo SDK 及板端视觉拆分 | 待实施；运行路径与环境检查、既有 43 项测试、FK/IK、标定配套哈希、可用离线回放 |
| 第三批 | MSPM0 示例、完整 TI SDK、平台共享文件、板端诊断、参考资料清理 | 待实施；两个 build.py / Keil 引用同时更新，有 Windows 工具链一侧重新编译，硬件资料完整保留 |

每批独立分支 / PR，合并由用户审查。代码迁移不混入 PID、速度、目标、阶段或控制策略变化，不自行运行真机。

## 当前 → 目标路径

下列为迁移清单；同一目录的剩余文件按“逐项分类”规则处理，不能按名称直接删除。目标文件名可在实施时收敛，职责及行为不变。

| 当前来源 | 目标 | 必须同步处理的依赖 |
| --- | --- | --- |
| `car-arm/hardware/k230_blue_pick.py`、`k230_blue_pick_onboard.py`、`move_ready.py`、`run_task.py`、`probe_linux_servos.py` | `host/apps/` | CLI、入口工作目录、进程锁、subprocess、原兼容命令 |
| `car-arm/hardware/task_controller.py`、`blue_pick_geometry.py`、`blue_pick_settle.py`、`gripper_geometry.py`、`motion_floor_envelope.py` | `host/arm/` | ROOT / sys.path、配置加载、FK、STL 路径；数值逻辑不改 |
| `car-arm/ros2_ws/src/arm_pkg/arm_pkg/{joint_mapping,urdf_fk}.py` | `host/arm/` | 所有导入和测试；撤去不完整 ROS2 工作区命名 |
| 同目录 `servo_bus.py` | `host/transport/` | PC pyserial SDK 路径、锁和读写调用 |
| 同目录 `scservo_sdk/` | `third_party/ftservo_python/` | 保留当前应答 ID / 载荷长度校验补丁、MIT 许可证，补来源与本地修改说明 |
| `arm/hardware/FTServo_Python-main/` | 对照后合并到上述第三方来源说明；原版由 Git 历史追溯 | 不让原版覆盖当前补丁，不混用 `machine.UART` 和 pyserial |
| `car-arm/ros2_ws/src/arm_pkg/urdf/` | `models/genkiarm/` | 当前 URDF 和全部 STL 字节不变；保留 `CCC.stl` / `CCC2.stl` 名称与引用 |
| `arm/robot_description/genkiarm.urdf` | 待归档旧参考，由整理前提交追溯 | 与运行 URDF 数值不同，不用于替换运行模型 |
| `car-arm/hardware/` 下六份 JSON：关节映射、PID、速度、准备位、task 策略、pick 配置 | `configs/arm/` | 路径字段 / Configuration / 测试同步；数值及语义不改 |
| `car-arm/k230/calibration/runtime/` | `configs/vision/runtime/` | 内外参与 provenance 成套迁移，更新引用，不重算标定 |
| `car-arm/k230/onboard/` | `device/k230/vision/` | 主机读取源码并拼接上传的方式及精确 import 替换必须适配 |
| `car-arm/k230/localize_board_faces.py`、`localize_pick_cube.py`、`runtime_cube_faces.py`、`cube_outline_pose.py`、`grasp_face_quality.py` | `host/vision/` | subprocess 入口、内外参路径、公共四角点 / 位姿函数 |
| `car-arm/k230/capture_board_features.py`、`capture_board_once.py`、`capture_pick_frame.py`、`download_calibration.py` 等共用传输部分 | `host/transport/`，工具入口留 `tools/capture/` | REPL、导出、源码拼接与采集工具导入；不按 capture 名称整文件归档 |
| `camera_preview.py`、`start_preview_verified.py`、`collect_raw_pair.py`、`record_grasp_pair.py`、`capture_calibration.py`、`capture_intrinsics_60s.py`、`download_calibration_fast.py` 等采集入口 | `tools/capture/` | 主机 / 板端执行位置逐段确认；IDE 路径可覆盖；仍不驱动舵机 |
| `audit_intrinsics.py`、`recheck_intrinsics.py`、`select_calibration.py`、`calibrate_from_cube_bottom_midpoint.py`、`car-arm/hardware/calibration/` | `tools/calibration/` | 输入输出参数、本地数据路径、与运行标定分离 |
| `analyze_*.py`、`compare_*.py`、`inspect_grasped_blue.py`、`validate_grasp_correspondences.py`、`replay_onboard_detector.py` | `tools/analysis/`，运行公共函数先抽至 `host/vision/` | 先断开运行代码对分析脚本的依赖；工具保留各自样本语义 |
| `car-arm/hardware/tests/`、`scripts/offline_check.py` | `host/tests/` 与根 `scripts/` 检查入口 | 工作目录、无采集数据运行、FK/IK 与参数一致性 |
| `car-arm/scripts/{host,vision,open_preview}`、requirements | 根 `scripts/` 与对应依赖清单 | 保留两套环境、解释器选择与可执行位；不提交本机安装目录 |
| `car/demo/01_motor_encoder/{main.c,demo_control.c,demo_control.h,ti_msp_dl_config.c,ti_msp_dl_config.h,keil/motor_demo.uvprojx,build.py}` | `device/mspm0/motor_encoder/` | 命令行构建、Keil include / library / startup / scatter 引用 |
| `car/demo/02_uart1_link/{main.c,build.py,keil/uart1_link.uvprojx}` | `device/mspm0/uart_link/` | 撤去依赖相邻 motor 示例的路径，保持源与构建语义 |
| `car/demo/01_motor_encoder/source/` | `third_party/ti_mspm0_sdk/` | 完整保留 1438 文件，两个示例同时改引用，不改成依赖本机 SDK |
| 电机示例 `keil/startup_mspm0g350x_uvision.s`、`keil/mspm0g3507.sct` | `device/mspm0/platform/` | 两份 build.py 与两个 Keil 工程一起更新 |
| `arm/examples/uart1_*.py`、`car/demo/02_uart1_link/k230_uart2_test.py` | `device/k230/diagnostics/` | 原生 `machine.UART` 代码独立保留；文档 / IDE 路径更新 |
| `car/硬件信息/` | `hardware/car/` | 全部 17 文件原样保留，相关链接更新 |
| `arm/` 内其他硬件资料 | `hardware/arm/`（按实有内容） | 不导入本机完整厂家包，不凭目录名生成重复资料 |
| 当前模块说明 / 旧审查 / Git 准备文档 | `docs/guides/` / `docs/history/` | 保持旧链接可查，历史不当作当前状态 |

## 先抽公共运行函数，再搬工具

`localize_board_faces.py`、`localize_pick_cube.py`、`runtime_cube_faces.py` 与 `grasp_face_quality.py` 当前从 `analyze_grasp_batch.py` 取 `ordered_quad` / `pose_candidates` / `face_candidates`。第二批先把这些函数抽成运行库，分析工具反向导入该库，再移动文件。不得删除“看起来只是历史分析”的运行依赖。

`capture_board_features.py` 读取 `onboard/blue_face_core.py` 和 `detect_blue_faces.py`，移除精确 import 行后拼接发送至 K230。因此迁移检查必须包括普通导入、subprocess、源码拼接、配置数据路径及 CLI 工作目录。

另有当前代码 / JSON 双重定义：闭合量在几何代码中为 `nominal-20`，阶段观察时间在执行器中为 6/6/8 秒，JSON 也记录相同配置。结构迁移先保留实际行为，后续再独立收敛成单一配置来源，不借迁移改运动策略。

## 归档 / 删除候选与保留边界

| 内容 | 处理方式 |
| --- | --- |
| 电机 `empty.c/.h`、`Hardware/`、平铺 `ti/`、旧 `tools/keil/` | 第三批核对 include、项目输入、构建钩子后再决定。平铺驱动有 13 个同名异内容文件，不能直接去重 |
| `empty.syscfg` | 有配置参考价值；尚未工具重新生成验证，不能声称等同当前手工配置 |
| 重复 SDK / 旧 URDF | 以已运行版本为准，保留许可、来源与差异摘要，旧版由 Git 历史追溯；本批不删除 |
| 原始采集、日志、依赖环境、构建输出、探针本机配置 | 继续本地忽略，不复制进新结构；迁移时同步检查忽略规则 |
| `.local_archive/`、旧 PID 实验 | 不复制、不恢复、不提交或上传 |
| 当前运行内外参、完整 STL、完整 TI SDK、小车 17 个资料 | 必须继续纳入 Git，不能按“历史”或“体积大”排除 |

## 后续迁移验收

1. 迁移前后记录运行配置数值、模型字节和标定哈希；路径字段只改引用。内参 SHA-256 为 `9bd4426442417c9425f146971464a4946463a3b5704095decc5d2ed7075f0e07`，外参为 `23c0b00a119b5c78dcbbd7a00b3a9b75828faffef07e5d21a03fe496cfbb75b3`。
2. 既有 43 项测试、默认离线检查与四阶段 FK/IK 通过；从不含本地采集数据的检出目录验证导入和路径。具备已保存图片 / 特征时做离线回放，缺样本明确记录。
3. 同时检查 shell 入口、源码拼接、进程锁、资源路径和 Git 忽略规则。当前控制锁虽在 `calibration/pid/batches/controller.lock`，仍是运行必需锁，不能当旧 PID 实验删除。
4. 两个 MSPM0 工程由具备 Windows Keil 的环境重新编译；Linux 静态检查不能替代此项。
5. 更新所有本地链接和入口文档。拟新增根命令 `./scripts/arm-ready`、`./scripts/pick-blue`、`./scripts/check`，尚未实现；旧入口暂时薄封装到同一实现，不维护第二套控制逻辑。
6. 只提交结构变化及必要兼容适配。准备位、抓取终态、PID、速度、加速度、同步发目标、标定参考定义全部保持原语义；不新增小车联动或硬件动作。
