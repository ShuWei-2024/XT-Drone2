# FAST-LIVO2 适配与局部地图

[文档索引](../../../docs/README.md) · [架构与接口](../../../docs/ARCHITECTURE.md)

本包提供 ROS 2 Jazzy 传感器输入适配、同步和局部地图门控。定位由独立 `astra_fast_livo2_core` C++ 包计算，Gazebo 真值不作为估计输入。

## 流程

```text
/sensors/lidar/points + RGB + CameraInfo + IMU
→ 输入检查 / 时间匹配 / 点时间适配
→ /fast_livo2/input/*
→ FAST-LIVO2 LIO/VIO
→ /fast_livo2/odom + 配准点云
→ local_mapping_guard
→ /local_mapping/obstacles + /local_mapping/ready
```

组合 launch 默认 `point_time_mode=snapshot`，针对 Gazebo 同时生成的整帧点云；RGB 匹配窗口 30 ms，要求 IMU 覆盖对应融合时刻。`synthetic_scan` 保留作对照，不能假定其为真实 Livox 点时间模型。实物必须使用真实点采样时间和标定。

LiDAR `time` 为预处理约定的微秒，Livox 等效消息 `offset_time` 为纳秒。`ring` 不是当前核心标准点云预处理的主要输入。核心测量时间与算法发布时间分开记录；pending VIO 不再为了处理已有 LIO 切片而额外等下一帧 LiDAR，LIO/VIO 仍按顺序更新共享状态。

## 当前启动

在仓库根目录构建并加载 `astra/env_117.bash`。完整传感器/PX4 任务使用 [快速启动](../../../docs/QUICKSTART.md) 的运行器，避免手动重复启动 clock、地图或控制节点。

单独算法调试可在已有传感器与仿真时钟下运行：

```bash
source astra/env_117.bash
ros2 launch astra_fast_livo2 fast_livo2.launch.py
```

当前 launch 保留 512 张灰度图像、最多 5000 个视觉点；视觉平面修正限幅可配置，standalone 默认关闭，隔离飞行运行器设为 0.04 m。准确参数见 [launch](launch/fast_livo2.launch.py) 和 [核心配置](../astra_fast_livo2_core/config/fast_livo2_gazebo.yaml)。不要把 launch 默认值与运行器覆盖值混用。

## 地图与保护

[local_mapping_guard.py](scripts/local_mapping_guard.py) 接收 odom 和配准点云，检查新鲜度、TF/帧和位姿跳变，限制局部体素半径/容量/保留时间，发布占据体素、膨胀障碍与 readiness。观测丢失时撤销地图，满足恢复条件后重新就绪。

静态下降保护柱策略是隔离场景选项，普通 3D 占据仍保留。室外地图范围、容量与膨胀由运行器配置；外部局部地图受限不表示 FAST-LIVO2 内部地图也已受限。核心 `map_sliding_en=false`，宽范围长期内存尚未证明平台化。

## 来源与诊断

ROS 2 核心来自 `U-AMC/FAST-LIVO2-ROS2` 提交 `a09599a288c56b832114d7ad9e898248a4111d9e`；vikit 来自 `Lmh-sys/rpg_vikit_ros2` 提交 `5b78f6536366182351fd5b95d1506340188dccf7`。原 FAST-LIVO2 ROS 1 源码仅作算法参考。

[早期集成记录](INTEGRATION_HISTORY.md) 保留历史实验；当前性能证据见 [延迟报告](../../validation/ODOM_DELAY_117.md)，当前飞行状态见 [验证索引](../../validation/README.md)。通过 `input_audit_csv` 与 `timing_audit_csv` 可记录输入与阶段时间；墙钟耗时包含调度等待，不等于 CPU 计算时间。
