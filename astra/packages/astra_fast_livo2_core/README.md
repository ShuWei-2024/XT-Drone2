# FAST-LIVO2 C++ 核心 · Astra Jazzy

[文档索引](../../../docs/README.md) · [输入适配与地图](../astra_fast_livo2/README.md)

本目录是 Astra 的实际 FAST-LIVO2 算法，移植自 `U-AMC/FAST-LIVO2-ROS2` 提交 `a09599a288c56b832114d7ad9e898248a4111d9e`，在 ROS 2 Jazzy 构建。感谢 HKU MARS、Chunran Zheng 与 ROS 2 移植维护者；原许可和来源保留。

## 使用与配置

从仓库根目录使用 `bash astra/build_117.bash` 构建，通过 `astra_fast_livo2/fast_livo2.launch.py` 或隔离 SITL 运行器启动。本工程的 Sophus/vikit 和消息接口已纳入构建，不按上游 Humble 教程另装全局依赖。

主仿真参数在 [fast_livo2_gazebo.yaml](config/fast_livo2_gazebo.yaml)。输入为适配后的标准 PointCloud2、RGB 和 IMU；估计状态、路径及配准点云由算法计算。

## 本地适配重点

- Gazebo snapshot 点时间和有界 IMU 传播；保留真实传感器配置入口。
- 测量时间戳转换、机体参考点及安装杆臂修正。
- 已有 LIO 切片的 pending VIO 不再额外等下一帧 LiDAR。
- 视觉特征/残差/条件数/位姿修正与计算/墙钟时间诊断。
- 灰度图像数量上限与可选视觉修正质量门控。

回归测试位于 `test/`，构建后运行 `ctest --test-dir build_117/astra_fast_livo2_core --output-on-failure`。该测试不替代运动精度、长期内存或硬件验证。

## 上游资料

[原始上游说明](README.upstream.md) 保留算法、论文、数据与原安装文档，其中 Humble/设备配置仅为参考。当前运行教程与状态以 [快速启动](../../../docs/QUICKSTART.md) 和 [验证索引](../../validation/README.md) 为准。
