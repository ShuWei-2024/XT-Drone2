# 文档索引

本页集中列出使用文档、模块入口与专题报告。首页提供常用构建、测试和启动命令；查详细流程时按下表直接进入对应条目。

## 使用与维护

| 文档 | 内容 |
| --- | --- |
| [安装教程](QUICKSTART.md) | 系统依赖、PX4/DDS Agent、ROS 构建、室外与室内地图准备 |
| [操作指南](OPERATIONS.md) | RViz2/Gazebo、航点任务、自动路线、运行记录与排障 |
| [架构与接口](ARCHITECTURE.md) | 各模块的输入、处理、输出，话题、坐标和控制权 |
| [开发规范](DEVELOPMENT.md) | 目录职责、回归方法、隔离运行、Git 工作流与许可 |
| [项目接手上下文](PROJECT_CONTEXT.md) | 项目目标、技术决策、当前状态与后续任务，供人员或 AI 接手 |
| [验证报告总览](../astra/validation/README.md) | 验证状态、历史报告、成功和失败证据及适用范围 |

## 模块入口

| 模块 | 说明或实现 |
| --- | --- |
| 仿真启动与进程隔离 | [SITL 运行器](../astra/tools/run_sitl_117.py) |
| Mid-360 / RGB / IMU 等效传感器 | [传感器模块说明](../astra/packages/astra_gz_sensors/README.md) |
| FAST-LIVO2 输入与 ROS 适配 | [适配器说明](../astra/packages/astra_fast_livo2/README.md) |
| FAST-LIVO2 算法与来源 | [核心模块说明](../astra/packages/astra_fast_livo2_core/README.md) |
| 局部地图与 readiness | [地图保护节点](../astra/packages/astra_fast_livo2/scripts/local_mapping_guard.py) |
| EGO-Planner 与移植改动 | [规划器来源与改动](../astra/packages/ego_planner_jazzy/UPSTREAM.md) |
| 共享安全边界 | [边界处理工具](../astra/tools/planning_envelope.py) |
| 轨迹采样与 Offboard 执行 | [执行模块说明](../astra/packages/astra_px4_offboard/README.md) |
| 起降与飞行监督 | [飞行监督器](../astra/tools/ego_sitl_flight.py) |
| External Vision | [EV 输入与融合门控](../astra/tools/external_vision_sitl.py) |
| RViz2 航点与中文窗口 | [任务状态管理](../astra/tools/rviz_mission.py)、[任务面板](../astra/tools/rviz_mission_panel.py) |

## 场景与验证专题

| 专题 | 报告 |
| --- | --- |
| 室外地图来源、准备与范围 | [Baylands 公园](../astra/validation/OUTDOOR_PARK_117.md) |
| RViz 网格、青色航点边界与几何精化扩区 | [航点范围与扩区](../astra/validation/GOAL_REGION_REFINEMENT_117.md) |
| 全图范围选点、连通空域与远端飞行 | [全图空域与验证](../astra/validation/WHOLE_MAP_AIRSPACE_117.md) |
| 全图远端往返与消息接收积压 | [事件执行器复测](../astra/validation/WHOLE_MAP_RECEIVER_117.md) |
| RViz 按开始后起飞前消息超时 | [交互起飞接收修复](../astra/validation/RVIZ_PREFLIGHT_RECEIVER_117.md) |
| RViz 转弯时重规划失败、预览撤销 | [重规划衔接修复](../astra/validation/REPLAN_BOUNDARY_117.md) |
| 室内真实扫描地图与绕障 | [Replica office4](../astra/validation/REAL_SCAN_AVOIDANCE_117.md) |
| RViz2 三维多航点交互 | [交互飞行验收](../astra/validation/RVIZ_GOAL_FLIGHT_117.md) |
| 独立通行／降落区、EGO 返航与官方 RTL | [返航策略与隔离验证](../astra/validation/TRANSIT_RETURN_117.md) |
| 点选后起飞但没有横向轨迹 | [边缘航点诊断与准入修复](../astra/validation/RVIZ_EDGE_GOAL_117.md) |
| 彩色场景复测 | [定位、延迟与箱体绕障](../astra/validation/APPEARANCE_RETEST_117.md) |
| odom 延迟分解 | [处理、传输与接收审计](../astra/validation/ODOM_DELAY_117.md) |
| 共享路线边界与 EV | [边界与融合验证](../astra/validation/SHARED_ENVELOPE_EV_117.md) |
| 同配置重复与失败清单 | [室外重复验证](../astra/validation/OUTDOOR_REPEAT_EV_117.md) |
| 复杂环境与长距离漂移 | [分航段漂移审计](../astra/validation/OUTDOOR_LONG_DRIFT_117.md) |

## 上游参考

- [XT-Drone2 原始介绍](README.upstream.md)
- [DDS 话题手册](DDS_Topics_Manual.md)

验证报告记录当时配置，不代表任意地图或实物已通过。原始模型、日志和视频位于本机 `astra/runtime/`，不随仓库分发。给其他 AI 接手时，先提供《项目接手上下文》，再按本页查找相关模块与证据。
