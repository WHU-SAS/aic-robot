# 第三方依赖与参考

- [ftservo_python](ftservo_python/README.md)：PC 当前运行 SDK，保留 MIT 许可证和应答校验补丁。bootstrap 仅将此运行 SDK 加入导入路径。
- `ti_mspm0_sdk/`：电机示例原 `source/` 的全部1438文件，逐字节迁移；保留内含 TI / CMSIS 等版权与许可，两份 MSPM0 工程共享。
- [reference](reference/README.md)：隔离的上游 SDK、旧 URDF 与未参与构建的模板。不是运行依赖，不加入 host 或构建搜索路径。

本次不更新第三方版本、不改协议、不裁剪 TI SDK。各目录许可只适用于对应文件，不扩展为全仓统一许可证。
