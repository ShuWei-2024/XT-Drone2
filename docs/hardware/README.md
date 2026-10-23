# 实物文档

[返回总索引](../README.md) · [仿真文档](../simulation/README.md)

实物线用于把已经在 SITL 中验证的接口迁移到真实飞控、RK3588 伴随计算机和真实传感器。每个阶段都必须保存日志并满足通过条件，不能因为仿真飞行成功就跳过硬件测试。

## 实施顺序

| 阶段 | 文档 | 输出 |
| --- | --- | --- |
| 1 | [项目框架](01_project_framework.md) | 设备分工、控制权和数据流 |
| 2 | [硬件信息](02_hardware.md) | 型号、接线、供电和安装记录 |
| 3 | [PX4 固件与烧录](03_px4_firmware.md) | 硬件 target、固件、参数基线和回滚方式 |
| 4 | [RK3588 环境](04_rk3588_setup.md) | OS、ROS 2、依赖、Agent 和启动环境 |
| 5 | [传感器驱动](05_sensor_setup.md) | Mid-360、D435i、IMU 话题和时间源 |
| 6 | [ROS 2 与 DDS 话题](06_ros_dds_topics.md) | 消息类型、发布者、订阅者、坐标和频率 |
| 7 | [标定与时间同步](07_calibration.md) | TF、外参、时间配对和高度基准 |
| 8 | [飞行前测试](08_preflight.md) | 通信、无桨、定位、External Vision 和 Offboard 门控 |
| 9 | [飞行实验](09_flight_tests.md) | 悬停、短距离、多航点和绕障记录 |
| 10 | [安全与故障恢复](10_safety_recovery.md) | 接管、失联、定位失败和降落行为 |
| 11 | [日志与排障](11_logs_troubleshooting.md) | ULog、ROS 日志、复现和回滚 |

## 不能直接从仿真继承的项目

- 传感器测量时间和驱动队列延迟；
- LiDAR、相机、IMU 与机体的实际外参；
- RK3588 温度、算力、内存和网络负载；
- 电机振动、供电噪声、机体响应和 PX4 参数；
- PX4 External Vision 的实际融合状态；
- 失联、低电量、定位丢失和人工接管行为。

实物先以官方 PX4 1.17 固件为基线。仿真构建存在 POSIX HRT、Gazebo 磁力计及 DDS 频率补丁，不能笼统称为“完全没有修改 PX4”；实物适用范围和构建边界见[固件与烧录](03_px4_firmware.md)。
