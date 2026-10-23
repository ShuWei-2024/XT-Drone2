# 文档索引

本页集中列出使用文档、模块入口与专题报告。项目文档分为两条部署线：**仿真线**用于 Gazebo/PX4 SITL 验收，**实物线**用于真实传感器、飞控和分级飞行迁移。两条线共享架构和 ROS 2 接口，但实物参数、外参、时间和安全结论必须重新确认。

## 仿真线

| 文档 | 内容 |
| --- | --- |
| [仿真文档总览](simulation/README.md) | 仿真阶段顺序和各专题入口 |
| [仿真安装与快速启动](QUICKSTART.md) | Ubuntu、ROS 2 Jazzy、PX4 1.17 SITL、Gazebo、地图和构建 |
| [仿真操作与排障](OPERATIONS.md) | RViz2/Gazebo、航点任务、自动路线、运行记录与故障定位 |
| [仿真架构与接口](ARCHITECTURE.md) | Gazebo 传感器、FAST-LIVO2、地图、规划、执行桥和 PX4 接口 |
| [仿真验证报告总览](../astra/validation/README.md) | SITL 成功、失败、适用范围和剩余验收项 |

## 实物线

| 文档 | 内容 |
| --- | --- |
| [实物文档总览](hardware/README.md) | 实物迁移阶段顺序和前置条件 |
| [实物文档总览](hardware/README.md) | PX4 1.17 固件、Agent 通信、真实传感器、外参、分阶段上电与飞行 |
| [DDS 话题手册](DDS_Topics_Manual.md) | PX4 uXRCE-DDS 输入输出消息和标准 Offboard 接口 |

## 共享开发与维护

| 文档 | 内容 |
| --- | --- |
| [架构与接口](ARCHITECTURE.md) | 两条部署线共用的数据流、话题、坐标、时间和控制权 |
| [开发规范](DEVELOPMENT.md) | 目录职责、回归方法、隔离运行、Git 工作流与许可 |
| [项目接手上下文](PROJECT_CONTEXT.md) | 项目目标、技术决策、当前状态与后续任务，供人员或 AI 接手 |

## 架构与证据

| 文档 | 内容 |
| --- | --- |
| [共享架构索引](architecture/README.md) | 仿真和实物共用的数据流、坐标、时间与控制权 |
| [仿真验证报告总览](../astra/validation/README.md) | SITL 验收、故障记录和证据边界 |

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
| 静态场景备用撤退与降落 | [连通恢复图](../astra/tools/connected_recovery.py) |
| 轨迹采样与 Offboard 执行 | [执行模块说明](../astra/packages/astra_px4_offboard/README.md) |
| 起降与飞行监督 | [飞行监督器](../astra/tools/ego_sitl_flight.py) |
| 飞行审计采样、增量写盘与上锁后导出 | [记录存储](../astra/tools/bridge_sample_store.py) |
| External Vision | [EV 输入与融合门控](../astra/tools/external_vision_sitl.py) |
| RViz2 航点与中文窗口 | [任务状态管理](../astra/tools/rviz_mission.py)、[任务面板](../astra/tools/rviz_mission_panel.py) |

## 场景与验证专题

| 专题 | 报告 |
| --- | --- |
| 2026-10-22 延迟、高度与地面原点三项失败分解 | [验收失败逐项检查](../astra/validation/ACCEPTANCE_FAILURES_20261022.md) |
| 远端断链返航、障碍旁停止与不可下降区撤退 | [多位置故障恢复](../astra/validation/MULTILOCATION_FAULTS_117.md) |
| 空中估计器重置、完整 DDS 断链与机载自恢复 | [重置与断链故障验收](../astra/validation/ESTIMATOR_LINK_FAULTS_117.md) |
| 2026-10-21 Baylands 固定配置收尾复测 | [固定配置收尾](../astra/validation/BAYLANDS_ACCEPTANCE_20261021.md) |
| 2026-10-21 Baylands 长时运动、Odom 超时与内存时间窗 | [长时运动中止审计](../astra/validation/BAYLANDS_LONG_MOTION_20261021.md) |
| 正常融合长路线、五分钟保持与同配置三轮重复 | [长路线重复验收](../astra/validation/NORMAL_LONG_ACCEPTANCE_117.md) |
| 仿真收尾范围、故障覆盖与实物前缺项 | [仿真收尾验收](../astra/validation/SIMULATION_ACCEPTANCE_117.md) |
| 审计缓存、长路线高度失败与关闭视觉更新的对照 | [审计内存与高度分解](../astra/validation/COMPACT_AUDIT_HEIGHT_117.md) |
| 图像时间偏移、LIO 空切片与 Odom 发布缺口 | [测量切片时间审计](../astra/validation/LIVO_CUT_TIMING_117.md) |
| 室外地图来源、准备与范围 | [Baylands 公园](../astra/validation/OUTDOOR_PARK_117.md) |
| 连通空域开放、附近降落与地图失联恢复 | [扩区与恢复验证](../astra/validation/CONNECTED_AIRSPACE_117.md) |
| RViz 网格、青色航点边界与几何精化扩区 | [航点范围与扩区](../astra/validation/GOAL_REGION_REFINEMENT_117.md) |
| 全图范围选点、连通空域与远端飞行 | [全图空域与验证](../astra/validation/WHOLE_MAP_AIRSPACE_117.md) |
| 全图远端往返与消息接收积压 | [事件执行器复测](../astra/validation/WHOLE_MAP_RECEIVER_117.md) |
| Python GC 暂停与 PX4 接收历史复测 | [接收暂停修复复测](../astra/validation/GC_RECEIVER_117.md) |
| 解锁前 EV／全局原点初始化退出 | [原点启动检查与有界等待](../astra/validation/EV_ORIGIN_STARTUP_117.md) |
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
