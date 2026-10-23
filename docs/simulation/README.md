# 仿真文档

[返回总索引](../README.md) · [实物文档](../hardware/README.md)

仿真线用于在 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 和 PX4 1.17 SITL 中验证软件链路。它验证的是接口、算法、规划和保护逻辑，不等同于真实传感器、机体动力学或实物安全认证。

## 文档顺序

| 阶段 | 文档 | 目的 |
| --- | --- | --- |
| 1 | [环境与依赖](01_environment.md) | 安装 ROS、Gazebo、PX4 和 DDS Agent |
| 2 | [构建与回归](02_build.md) | 编译、消息检查和自动化测试 |
| 3 | [启动与操作](03_startup.md) | 启动 Gazebo、RViz2、地图和任务窗口 |
| 4 | [规划与执行](04_operation.md) | 航点、多航点、绕障和保护行为 |
| 5 | [验收与证据](05_validation.md) | 读取运行目录、审计报告和验收边界 |
| 6 | [故障排查](06_troubleshooting.md) | 按日志和失败阶段定位问题 |

## 现有详细文档

- [仿真安装与快速启动](../QUICKSTART.md)
- [仿真操作与排障](../OPERATIONS.md)
- [验证报告总览](../../astra/validation/README.md)

## 当前链路

```text
Gazebo 传感器
  -> FAST-LIVO2
  -> 局部障碍物地图与 readiness
  -> EGO-Planner
  -> 轨迹检查与保护执行桥
  -> PX4 SITL
```

仿真结论必须带上场景、参数、路线和运行证据；“一次飞通”不能自动扩展为任意地图和实物通过。
