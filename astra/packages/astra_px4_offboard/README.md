# 轨迹预览与受保护 PX4 执行桥

[文档索引](../../../docs/README.md) · [架构与接口](../../../docs/ARCHITECTURE.md)

## 流程与职责

```text
EGO /planning/bspline
→ preview_bspline：曲线检查、20 Hz 采样、有效/保持状态
→ guarded_bspline_bridge：测量配对、camera_init → PX4 NED、保护
→ /x500_0/fmu/in/trajectory_setpoint
```

预览检查地图 readiness、规划心跳、轨迹有效性/时间、速度与加速度；本身不输出 `/fmu/in/*`。桥独立启动默认 `enable_px4_output=false`，只发诊断预览。启用输出后也不自行解锁或进入 Offboard；隔离 SITL 任务监督器通过健康检查后管理起飞和控制权交接。

## 使用

完整闭环使用 [快速启动](../../../docs/QUICKSTART.md) 和 [操作指南](../../../docs/OPERATIONS.md) 的统一运行器。独立无控制输出预览：

```bash
source astra/env_117.bash
ros2 launch astra_px4_offboard preview_bspline.launch.py
# 另一个已加载环境的终端
ros2 launch astra_px4_offboard guarded_bspline_bridge.launch.py
```

已有传感器/地图/规划/PX4 状态才会形成有效预览。`sitl_only` 是操作配置声明，不是自动识别真实飞控的技术保证；不要把这些入口连接实物。

## 时间、坐标与失效处理

桥保留有界 odom/PX4 历史，选择同 reset epoch 且测量配对、新鲜度都通过的位姿；同时检查当前状态新鲜度，避免回调先后顺序误判同步。坐标转换采用冻结标定，包含系统机体参考约定；不使用 Gazebo 真值生成 setpoint。

运行器把经过地面验证的高度偏移用于监督器和执行桥，避免二者使用不同零点。reset、航向、两估计器差值、跟踪误差、ready/轨迹失效等检查持续生效。冻结 tracking yaw 是可选策略，不能补偿真实航向估计漂移。

故障进入锁存保持，冻结新鲜 PX4 位姿；默认持续 1 秒后请求降落，保持请求直到进入降落或上锁。任务监督器也处理地图/EV/界面丢失、边界和电池故障。单机只有一个当前 setpoint 控制权拥有者。

## 验证与参考

最新 EV、长路线与重复结果见 [验证索引](../../validation/README.md)。[早期坐标与执行实验](VALIDATION_HISTORY.md) 保留旧对照，不能将其“尚未接入”状态用于描述当前链路。

`audit_frame_alignment.py` 是只读测量时间审计工具；当前精度判定以冻结地面参考和运行报告为准。PX4 消息话题由 [px4_topics.py](scripts/px4_topics.py) 按版本/命名空间生成。
