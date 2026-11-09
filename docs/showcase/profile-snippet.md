### 思达｜视觉追踪与自主抓取机器人

基于K230、MSPM0与六轴机械臂，围绕20 mm蓝色方块，实现语音触发、视觉追踪、底盘接近、停车定位及抓取，并已演示目标重搜和夹取失败重试。

**技术方向：** 嵌入式视觉、几何定位、编码器闭环控制、机械臂运动学与车臂协同。

<p>
  <img src="https://raw.githubusercontent.com/WHU-SAS/aic-robot/main/docs/showcase/images/03-parking-window.png" width="460" alt="停车区域与方块底面中心示意，使用历史配置">
  <img src="https://raw.githubusercontent.com/WHU-SAS/aic-robot/main/docs/showcase/images/09-cube-model-projection.png" width="320" alt="五个测量角点与20mm立方体模型重投影">
</p>

[粗定位演示（9.2秒，原速）](https://github.com/WHU-SAS/aic-robot/blob/main/docs/showcase/videos/blue-cube-coarse-localization.mp4) · [项目仓库](https://github.com/WHU-SAS/aic-robot) · [更多图片与说明](https://github.com/WHU-SAS/aic-robot/tree/main/docs/showcase)

视频为K230原片经电脑后期叠加检测框与估距；图片包含算法标注及历史示意。重复运行稳定性与精度仍在验证。
