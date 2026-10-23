# 仿真故障排查

[返回仿真索引](README.md)

先按失败阶段查证据：

| 阶段 | 首先检查 |
| --- | --- |
| 启动前 | `preflight_health.json`、传感器话题、地图 readiness |
| 起飞后 | `motion_events.json`、PX4 状态、odom 测量年龄 |
| 规划失败 | `planner.log`、目标准入、障碍物点云和轨迹有效性 |
| 轨迹中止 | 执行桥状态、规划心跳、TF、PX4 failsafe |
| 高度/坐标异常 | 测量时间配对、冻结高度参考、External Vision 审计 |
| 进程退出 | 运行器日志、所属进程、DDS 和 Gazebo 分区 |

不要先放宽超时或关闭保护；先区分算法计算、消息排队、传输延迟、时间戳错误和控制器状态。
