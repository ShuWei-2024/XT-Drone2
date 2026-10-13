# PX4 1.17 对 Astra 工程的兼容性评估

日期：2026-10-10。此次仅静态核对本地工程、官方文档与指定版本源码，未切换飞控、未编译或启动 PX4 1.17。

**可迁移，主要工作集中在 PX4 通信、仿真接入和飞行验证。没有证据表明需要重写 FAST-LIVO2 或 EGO-Planner 核心。新版本不保证自动解决现有定位/控制偏差。**

## 版本证据

- 本地 PX4 为 `v1.15.3-dirty`。本地定时器实现和对应回归测试存在修改，升级前必须保存，不能直接覆盖。
- 官方仓库存在 `v1.17.0` 标签；`px4_msgs` 存在 `release/1.17` 分支。升级应锁定标签和消息提交，不能使用已继续演进的 `main` 作为“1.17”。
- 官方当前 ROS 2 指南推荐 Ubuntu 24.04 / ROS 2 Jazzy，与我们当前系统一致。1.17 发布说明指出 Gazebo Harmonic 仍为受支持、CI 测试的版本；无需为了 PX4 同时升级 Gazebo。
- ROS 2 消息版本化及转换节点从 1.16 起引入，1.17 包含该机制。新版本增强了 ROS 2 能力，但 1.15 本来也支持原生 ROS 2/uXRCE-DDS。

## 各模块影响

| 模块 | 预期影响 | 要做的工作 |
| --- | --- | --- |
| FAST-LIVO2 | 小：输入为 PointCloud2、Image、Imu，输出为 ROS odom/点云 | 保持传感器话题、外参与测量时间语义；复测频率和延迟 |
| 局部地图 | 小：主要读取标准 ROS 位姿与点云 | 复测 readiness、TF、新鲜度及故障撤销 |
| EGO-Planner 核心 | 小：读取地图/odom，输出 B-spline，与 PX4 版本隔离 | 保持规划接口；重新验收闭环避障 |
| PX4 执行桥与审计器 | 明确需要适配 | 重建匹配消息，调整版本化话题，核对消息字段、QoS、状态与重置逻辑 |
| Gazebo 飞机模型 | 预计可复用 Harmonic 传感器和场景，但尚未编译验证 | 核对 x500 模型、插件、启动参数、PX4 Gazebo bridge 与传感器输入 |
| 参数、EKF、控制与 failsafe | 需重新验收 | 按名迁移经过核对的参数，不直接使用旧 rootfs/参数 BSON 作为新版本基线 |

## 已发现的具体接口差异

对比当前本地消息与 `px4_msgs release/1.17`：

1. `VehicleStatus` 为 `MESSAGE_VERSION=1`，新增/移除部分常量和字段。`arming_state`、`nav_state`、当前使用的 Offboard/降落常量仍存在。
2. `VehicleLocalPosition` 为 `MESSAGE_VERSION=1`，地面距离相关字段变化，包括删除 `hagl_max`、增加 `hagl_max_z`/`hagl_max_xy`、距离方差和重置字段。当前桥使用的 x/y/z、heading、timestamp_sample、有效标志和 XY/Z/heading reset 字段仍存在。
3. `TrajectorySetpoint` 为版本 0，现有位置、速度、加速度、yaw 等接口保持。`OffboardControlMode` 数据定义在此对比中无变化；`VehicleCommand` 版本 0，现用起飞控制相关命令仍存在。`TimesyncStatus` 本轮对比没有实质定义差异。
4. 根据官方版本命名规则，版本 0 不加后缀，非零版本添加 `_v<version>`。因此直接使用 1.17 原生消息时，需处理 `/fmu/out/vehicle_status_v1`、`/fmu/out/vehicle_local_position_v1`；`/fmu/in/trajectory_setpoint` 等版本 0 接口仍保持原名。最终以实际运行话题确认。
5. 当前多个脚本把无后缀话题写死，且工作区有针对 1.15 构建的消息 overlays。仅更换 PX4 二进制而继续 source 旧消息，会造成接口不匹配；编译成功不能证明 DDS 消息可交换。
6. 1.17 默认 DDS 配置对 local_position 限制为 50 Hz、vehicle_status 为 5 Hz、TimesyncStatus 为 10 Hz。必须重新测量订阅频率、时间配对、新鲜度和故障反应，不沿用旧日志采样假设。

优先采用匹配的 1.17 消息和话题，不必为了兼容旧消息先增加转换节点。若确实需要同时兼容旧应用，可评估官方 message translation node；这会多一个进程和故障环节。

## 与当前不稳定问题有关的变化

- 1.17 发布说明包含 EKF 磁航向起飞后重置及 gyro bias 更新相关调整，值得在我们出现过的初始航向分歧场景中验证，不能仅凭发布说明宣布它已修复本项目问题。
- 新增气压计针对 GNSS 高度的自动标定行为，需按当前实际高度参考评估。现配置主要使用气压高度，不能把此功能直接解释成必然改善 Z 误差。
- 发布说明中有 Offboard 到 Position/hold 在无 RC 时的模式切换变化。我们目前采用保持 setpoint 再请求降落，仍需复测故障保持、降落接收与解除解锁。
- 本地 `drv_hrt.cpp` 的重复入队防护修补不在此次取回的 1.17 同文件中：该文件与未修改的 1.15.3 内容完全相同。这不证明 1.17 一定复现旧故障，但说明不能假设新版本已包含该补丁。应先跑原定时器回归，再决定是否移植；保留旧工程作为可恢复基线。
- 当前 Agent 源码为 v3.0.2。Agent 与 client 的协议/版本组合仍需核对和实际握手验证，不因 PX4 版本数字增加而自动判定相容或不相容，也不同时盲目换 middleware。

## 建议迁移顺序

现在可以建立独立的 1.17 验证环境，无需继续把所有调参投入旧版本。保留 1.15 的源码、补丁、日志和运行基线，单独使用 1.17 PX4 checkout、build/rootfs 和 ROS 消息安装前缀，避免旧 overlay 覆盖新消息。

1. 检查原生消息和 Agent 握手；先不解锁，确认 position/status/time sync 订阅和命令接口。
2. 验证 Gazebo 传感器、测量时间、定时器回归及重置记录。
3. 起飞、短距离前进/侧移、返回和降落；此阶段不让规划器驱动飞行。
4. 接回现有保护执行桥，复测当前两次往返绕障路线及地图退出/数据过期保护。
5. 在 1.17 上重新评估 XY/Z/航向偏差、实际障碍间距和长时间内存；旧版本的通过结果只作为对照。

此次评估不同时更换 FAST-LIVO2、规划器、Gazebo 或 XTDrone2 框架，以便将变化归因于 PX4 升级及其接口适配。

## 来源与保存材料

- [PX4 1.17 发布说明](https://docs.px4.io/main/en/releases/1.17.html)
- [官方 ROS 2 指南](https://docs.px4.io/main/en/ros2/user_guide.html)
- [ROS 2 消息版本转换规则](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html)
- [px4_msgs release/1.17](https://github.com/PX4/px4_msgs/tree/release/1.17)
- [PX4 v1.17.0 DDS 配置](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/uxrce_dds_client/dds_topics.yaml)
- [PX4 v1.17.0 定时器源码](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/platforms/posix/src/px4/common/drv_hrt.cpp)

原始文档、指定版本消息定义及比较结果保存在 `work/px4_117_compatibility_20261010/`。文档 main 会继续更新，迁移实施应再次核对指定 tag/提交。本次兼容性判断为静态评估，尚无 1.17 的构建或飞行通过证据。
