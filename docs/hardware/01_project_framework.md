# 实物项目框架

[返回实物索引](README.md)

```text
Mid-360 / D435i / IMU
          |
          v
RK3588：ROS 2 + FAST-LIVO2
          |
          +--> 局部障碍物地图 + readiness
          |
          +--> odom / 可选 External Vision
          v
EGO-Planner -> 轨迹检查与执行桥 -> uXRCE-DDS -> PX4
```

职责边界：

| 设备/模块 | 主要职责 |
| --- | --- |
| 真实传感器 | 产生 LiDAR、RGB、Depth、IMU 数据 |
| RK3588 | 驱动、FAST-LIVO2、局部地图、规划器和 ROS 2 Agent |
| PX4 | 姿态/位置控制、模式管理、动力输出和 failsafe |
| 人工遥控/急停 | 最终接管和停止飞行 |

正常控制权只有一个：执行桥发布规划 setpoint，PX4 负责跟踪。地图或 odom 失效时，执行桥和 PX4 的保护逻辑必须优先于旧轨迹继续执行。
