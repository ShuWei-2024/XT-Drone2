# 共享架构文档

[返回总索引](../README.md) · [仿真文档](../simulation/README.md) · [实物文档](../hardware/README.md)

这些内容描述仿真和实物共同使用的软件边界。部署环境不同，但消息契约、坐标约定、时间语义和安全控制权必须保持一致。

| 文档 | 内容 |
| --- | --- |
| [完整架构与接口](../ARCHITECTURE.md) | 模块流程、ROS 2 话题、坐标、时间和保护策略 |
| [Offboard 接口与切换](OFFBOARD.md) | 官方消息、预发送、模式/解锁确认及项目保护边界 |
| [进程、线程与 ROS 2 职责](PROCESSES.md) | 核心 5/6 个业务进程、完整仿真清单与实际数量查询 |
| [DDS 话题手册](../DDS_Topics_Manual.md) | PX4 uXRCE-DDS 输入输出消息 |
| [项目接手上下文](../PROJECT_CONTEXT.md) | 当前目标、决策、证据和未完成项 |

## 共同数据流

```text
传感器 -> FAST-LIVO2 -> 局部地图/readiness -> EGO-Planner
                                      -> 轨迹检查/执行桥 -> PX4
```

仿真使用 Gazebo 传感器和 PX4 SITL；实物使用 Mid-360、D435i、真实 IMU、RK3588 和飞控。实物必须重新验证驱动、外参、时间、算力和保护动作。
