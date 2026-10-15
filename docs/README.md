# 文档索引

按目的选择一份文档开始阅读。安装、操作、架构各有独立职责；本页集中导航，首页保留常用启动、构建与测试命令。

## 使用与维护

| 文档 | 内容 |
| --- | --- |
| [安装教程](QUICKSTART.md) | 系统依赖、PX4/DDS Agent、ROS 构建、室外与室内地图准备 |
| [操作指南](OPERATIONS.md) | RViz2/Gazebo、航点任务、自动路线、运行记录与排障 |
| [架构与接口](ARCHITECTURE.md) | 各模块的输入、处理、输出，话题、坐标和控制权 |
| [开发规范](DEVELOPMENT.md) | 目录职责、回归方法、隔离运行、Git 工作流与许可 |
| [项目接手上下文](PROJECT_CONTEXT.md) | 项目目标、对话中的技术决策、当前状态与后续任务，供人员或 AI 接手 |
| [验证报告索引](../astra/validation/README.md) | 指定配置与路线的指标、成功和失败证据、适用范围 |

## 模块源码入口

流程和接口统一查阅上面的《架构与接口》；需要修改实现时按下表进入。模块目录保留各自 README 和上游来源说明。

| 模块 | 目录或主要文件 |
| --- | --- |
| 仿真启动与传感器 | `astra/tools/run_sitl_117.py`、`astra/packages/astra_gz_sensors/` |
| FAST-LIVO2 与输入同步 | `astra/packages/astra_fast_livo2/`、`astra/packages/astra_fast_livo2_core/` |
| 局部地图与 readiness | `astra/packages/astra_fast_livo2/scripts/local_mapping_guard.py` |
| EGO-Planner 与共享安全边界 | `astra/packages/ego_planner_jazzy/`、`astra/tools/planning_envelope.py` |
| 轨迹执行与飞行监督 | `astra/packages/astra_px4_offboard/`、`astra/tools/ego_sitl_flight.py` |
| External Vision | `astra/tools/external_vision_sitl.py` |
| RViz2 航点交互 | `astra/tools/rviz_mission.py`、`astra/tools/rviz_mission_panel.py` |
| 场景准备与离线审计 | `astra/tools/prepare_*.py`、`astra/tools/audit_*.py` |

## 上游参考

- [XT-Drone2 原始介绍](README.upstream.md)
- [DDS 话题手册](DDS_Topics_Manual.md)

验证报告记录当时配置，不代表任意地图或实物已通过。原始模型、日志和视频位于本机 `astra/runtime/`，不随仓库分发。给其他 AI 接手时，提供《项目接手上下文》，再按本页查找所需文档与源码。
