# 文档索引

本索引是 Astra 分支的统一入口。主使用环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、PX4 v1.17.0。最后整理：2026-10-14。

## 按任务查找

| 任务 | 文档 | 内容 |
| --- | --- | --- |
| 首次安装、构建、启动 | [快速启动](QUICKSTART.md) | 固定依赖、PX4、DDS Agent、ROS 包、地图准备 |
| 理解模块如何连接 | [架构与接口](ARCHITECTURE.md) | 模块流程、话题、坐标、控制权、保护职责 |
| 点选航点、自动路线、分析失败 | [操作指南](OPERATIONS.md) | RViz2、地图、运行产物、审计、故障排查 |
| 修改代码、验证、提交 | [开发指南](DEVELOPMENT.md) | 构建/测试、版本管理、证据与许可 |
| 把项目交给其他人或 AI | [项目接手上下文](PROJECT_CONTEXT.md) | 技术背景、已做决策、当前成果、未解决问题与接手提示 |
| 查实际飞行结果 | [验证索引](../astra/validation/README.md) | 最新状态、各场景报告、历史证据 |

## 按模块查找

每个模块的“输入 → 处理 → 输出”和源码入口在 [架构与接口](ARCHITECTURE.md) 中统一说明；细节按下表进入。

| 模块 | 文档 / 源码入口 |
| --- | --- |
| XT-Drone2 / PX4 仿真启动 | [快速启动](QUICKSTART.md)、[运行器](../astra/tools/run_sitl_117.py)、[框架启动包](../xtd2_launch) |
| Mid-360 / D435i 等效传感器 | [传感器包](../astra/packages/astra_gz_sensors/README.md)、[传感器模型](../astra/packages/astra_gz_sensors/models) |
| 输入同步 / FAST-LIVO2 | [适配器说明](../astra/packages/astra_fast_livo2/README.md)、[核心源码与来源](../astra/packages/astra_fast_livo2_core/README.md)、[延迟审计](../astra/validation/ODOM_DELAY_117.md) |
| 局部地图 / readiness | [架构与接口](ARCHITECTURE.md)、[地图保护节点](../astra/packages/astra_fast_livo2/scripts/local_mapping_guard.py) |
| EGO-Planner / 共享边界 | [规划器来源与改动](../astra/packages/ego_planner_jazzy/UPSTREAM.md)、[共享边界与 EV 验证](../astra/validation/SHARED_ENVELOPE_EV_117.md) |
| B-spline 采样 / Offboard | [执行桥说明](../astra/packages/astra_px4_offboard/README.md)、[飞行监督器](../astra/tools/ego_sitl_flight.py) |
| External Vision | [架构与接口](ARCHITECTURE.md)、[EV 节点](../astra/tools/external_vision_sitl.py)、[重复融合验证](../astra/validation/OUTDOOR_REPEAT_EV_117.md) |
| RViz2 / 多航点任务 | [操作指南](OPERATIONS.md)、[交互飞行验收](../astra/validation/RVIZ_GOAL_FLIGHT_117.md) |
| 场景转换 / 地图准备 | [快速启动](QUICKSTART.md)、[室内扫描绕障](../astra/validation/REAL_SCAN_AVOIDANCE_117.md)、[室外地图](../astra/validation/OUTDOOR_PARK_117.md) |
| 真值配对 / 漂移 / 轨迹审计 | [操作指南](OPERATIONS.md)、[长距离漂移报告](../astra/validation/OUTDOOR_LONG_DRIFT_117.md)、[审计工具](../astra/tools) |

## 文档层级

- **使用与设计文档**：本目录的快速启动、架构、操作、开发说明，描述当前入口。
- **接手上下文**：对项目对话中的技术决策作整理，不是原始聊天记录，也不包含密码或个人信息。
- **验证报告**：记录指定日期、配置和路线的结果；早期失败保留，不表示当前代码仍有同一故障。以最新报告确定当前状态。
- **上游参考**：[原始框架介绍](README.upstream.md)、[DDS 手册](DDS_Topics_Manual.md)及各依赖的来源说明；不替代 Astra 的锁定版本教程。

修改入口、接口或默认行为时同步更新对应说明；修改验证结论时必须附运行名称、指标和证据。不要通过只保留成功记录来提高表面完成率。
