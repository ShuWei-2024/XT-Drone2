# Offboard 接口与模式切换

[返回架构索引](README.md) · [实物通信](../hardware/06_ros_dds_topics.md)

## 谁定义接口

Offboard 是 PX4 的飞行模式，不是一种由本项目发明的通信协议。PX4 定义控制消息、字段含义、模式编号和接收条件；DDS / DDS-XRCE 由 OMG 标准定义。Micro XRCE-DDS Agent 是 eProsima 的实现。

本项目定义的是：B-spline 如何采样、规划坐标如何转换到 PX4 NED、控制权如何交接，以及何时拒绝轨迹、保持或请求恢复。

```text
EGO B-spline
  -> 本项目采样、坐标转换和健康门控
  -> PX4 官方消息（px4_msgs）
  -> ROS 2 / DDS -> Micro XRCE-DDS Agent
  -> 串口或 UDP 上的 DDS-XRCE -> PX4 uXRCE-DDS Client
  -> PX4 uORB / Commander / 控制器
```

ROS 2 发布 VehicleCommand 并不是在该连接上发送 MAVLink 数据包。命令编号沿用 MAVLink 命令定义，但此处传输的是 PX4 官方 DDS 消息。遥控器或地面站也可以请求模式切换，本页说明项目中的程序化切换路径。

## 当前代码如何切换

源码入口：[起飞监督](../../astra/tools/ego_sitl_flight.py)、[消息发送](../../astra/tools/px4_calibration_flight.py)、[轨迹桥](../../astra/packages/astra_px4_offboard/scripts/guarded_bspline_bridge.py)。

```text
地面健康检查通过
  -> prestream 2 秒：持续发送当前位置 setpoint 和 OffboardControlMode
  -> VehicleCommand：DO_SET_MODE，param1=1、param2=6
  -> VehicleCommand：COMPONENT_ARM_DISARM，param1=1
  -> 等待 VehicleStatus 确认 Offboard 和已解锁
  -> 执行受控爬升，再交接给规划轨迹
```

`1` 表示启用 custom mode，`6` 是 PX4 的 OFFBOARD custom main mode 编号；它不是 `VehicleStatus.nav_state` 的数值。发命令只是请求，不能把 publish 成功视为 PX4 已接受。当前起飞流程等待状态确认；通用实物实现还应记录 VehicleCommandAck 的接受/拒绝原因，并核实状态。

项目按约 20 Hz（50 ms 一次）发送，不是等 FAST-LIVO2 或 EGO 每算完一帧才发一次。规划更新和 setpoint 发送是不同环节；输入过期时由保护逻辑阻止继续跟踪。

PX4 官方使用持续控制模式信号判断外部控制是否存活，文档要求以高于 2 Hz 的频率发送并在切换前预发送至少 1 秒。此要求不能理解成“500 ms 的定位延迟一定安全”。锁定版本的实际可用性检查还依赖 `COM_OF_LOSS_T`、所选控制模式和估计状态；本项目自己的 250 ms 等新鲜度检查属于另一层保护。

## 官方消息与本项目使用方式

以下是逻辑话题名。当前 SITL 使用 `/x500_0` 命名空间；消息版本可能增加 `_v1` 等后缀，由项目 `px4_topics.py` 根据消息版本处理。实物必须核对真实命名空间和消息版本。

| 话题 | 关键字段 | 本项目用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `timestamp`、`position` 等模式位 | 当前 `position=true`，其余模式位默认 false |
| `/fmu/in/trajectory_setpoint` | `position`、`velocity`、`acceleration`、`yaw` | NED 位置目标；可选速度前馈；未使用的目标字段设 NaN |
| `/fmu/in/vehicle_command` | `command`、`param1..7`、target/source | 请求模式、解锁、Land 等；坐标和含义按各命令定义 |
| `/fmu/out/vehicle_command_ack` | `command`、`result` | 确认命令接受、拒绝或处理中 |
| `/fmu/out/vehicle_status` | `nav_state`、`arming_state` | 确认实际模式和解锁状态 |
| `/fmu/out/vehicle_local_position` | `timestamp_sample`、XYZ、速度、有效性、reset 计数 | 跟踪、坐标配对和健康检查 |
| `/fmu/in/vehicle_visual_odometry` | 位姿、时间、协方差及坐标枚举 | 独立 EV 输入，不负责切换 Offboard |

NED 的轴是北、东、下，位置单位米，速度米/秒，yaw 用弧度。消息时间字段单位为微秒，须与 PX4 所需时间基准正确转换；不能把未同步的主机 Unix 时间或 SITL `/clock` 原样移到实物。检查时间同步、QoS、消息定义、DDS topic 配置后再发送。

## 项目保护与 PX4 failsafe 的区别

| 层 | 检查 | 行为归属 |
| --- | --- | --- |
| 规划/执行端 | odom、地图、轨迹心跳、跟踪误差、时间配对 | 项目代码拒绝继续跟踪，按监督策略交接保护 |
| PX4 | 外部控制信号、估计有效性、电池、遥控/其他 failsafe | 飞控依照参数选择退出模式及恢复动作 |

Offboard 丢失并不天然等于安全悬停：位置估计不可用时悬停可能无法执行。实物必须验证所选 failsafe 参数、接管路径与场地条件。

## 官方与本地依据

- [PX4 官方 Offboard 说明](https://docs.px4.io/main/en/flight_modes/offboard.html)（在线 main 随版本变化）。
- [PX4 官方 ROS 2 Offboard 示例](https://docs.px4.io/main/en/ros2/offboard_control.html)。
- [固定提交的 Offboard 可用性检查](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp)。
- PX4 消息源码位于独立 PX4 目录；ROS 2 定义位于项目 `xtd2_third_party_pkgs/px4_msgs` 子模块。两者必须匹配。
