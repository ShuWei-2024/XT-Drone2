# ROS 2、uXRCE-DDS 与话题

[返回实物索引](README.md) · [DDS 话题手册](../DDS_Topics_Manual.md)

Offboard 是 PX4 官方飞行模式，消息定义和模式切换由 PX4 规定；本项目实现轨迹采样、坐标转换及保护。详细流程见[Offboard 接口与模式切换](../architecture/OFFBOARD.md)，运行组织见[进程与线程](../architecture/PROCESSES.md)。

```text
ROS 2 节点
  <-> px4_msgs
  <-> Micro XRCE-DDS Agent（RK3588）
  <-> USB / UART / UDP
  <-> uXRCE-DDS Client（PX4）
  <-> PX4 uORB
```

| 方向 | 典型话题 | 用途 |
| --- | --- | --- |
| 传感器 -> FAST-LIVO2 | 项目实际 LiDAR、RGB、IMU 输入 | 定位和建图 |
| FAST-LIVO2 -> 地图 | `/fast_livo2/odom`、配准点云 | 局部障碍物和 readiness |
| 规划 -> 执行桥 | `/planning/bspline`、轨迹有效性 | 轨迹检查和采样 |
| ROS 2 -> PX4 | `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint` | Offboard 控制 |
| PX4 -> ROS 2 | `/fmu/out/vehicle_status`、`vehicle_local_position` | 状态、位置和保护 |
| 可选 EV | `/fmu/in/vehicle_visual_odometry` | 视觉里程计输入 |

每个话题都要记录发布者、订阅者、消息类型、坐标系、时间戳含义、实际频率和失效动作。话题能 `echo` 只能证明消息可见，不能证明时间、坐标和控制逻辑正确。
