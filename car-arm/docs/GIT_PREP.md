# Git准备（2026-10-13）

> 历史记录：保留 2026-10-13 项目分离 / 导入时期的路径与检查范围。当前工作目录为团队仓库 `aic-robot/car-arm/`；当前进度与后续布局见 [状态表](../../docs/status.md) 和 [整理计划](../../docs/reorganization.md)。

## 纳入版本控制

- 当前K230/PC抓取程序、采集/标定工具、控制器与离线测试。
- 用户选定的PID、运动配置、关节映射、抓取设置。
- `k230/calibration/runtime/`配套内外参快照及来源哈希。
- SDK及原MIT许可、FK/URDF/STL模型、依赖清单、当前使用说明。

## 仅保留本地

照片、配对编码与历史分析、运行日志、视觉诊断回放、已安装依赖、编辑器设置、旧文档和来源迁移记录均由`.gitignore`排除。数据留在原位置；不把旧PID实验纳入此项目或Git。

## 清理及可移植性

删除可再生Python缓存；未使用的ROS2占位manifest已本地归档，现目录只直接提供Python模块与模型。当前标定从原文件逐字节固化，硬件控制数值和流程不变。

`scripts/host`优先`.venv-host`，`scripts/vision`优先`.venv-vision`；本机无这两目录时沿用系统Python和`.vision_deps`。安装命令见README。`scripts/offline_check.py`默认不依赖未提交采集数据，历史完整采集复核需`--with-local-data`。

Git准备完成后可先查看`git diff --cached --stat`和`git status --short`。提交作者、远程地址及上传由用户后续指定；本次不创建远程、不推送。


## 完成状态

已初始化本地main分支并暂存。暂存文件不包含照片、实验记录、日志、依赖环境、缓存或旧PID实验。核心模型STL约55MB，保留用于FK与指尖几何。

只从暂存区导出的干净目录通过默认离线检查、43项单元测试、运行环境检查和当前30°/15mm/+10mm四阶段IK。未携带历史采集数据；视觉测试使用本机已有依赖，未重新联网安装。PID/运动数值和配套标定字节与整理前一致，文档链接及git diff --cached --check通过。详见[验证记录](git_preparation_20261013.json)。

以上记录描述独立项目导入前的整理阶段。独立项目已提交为445a2b9；本模块现导入WHU-SAS/aic-robot的car-arm目录，新增分支通过PR合并。
