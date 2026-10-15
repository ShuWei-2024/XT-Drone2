# 验证索引与当前状态

[总文档索引](../../docs/README.md) · [运行与审计方法](../../docs/OPERATIONS.md)

更新：2026-10-15。报告记录当时版本与场景，历史失败不删除；判断当前状态先阅读边缘航点、彩色复测、长距离漂移与重复验证报告。

## 当前状态

| 项目 | 结论 | 主要证据 |
| --- | --- | --- |
| PX4 1.17 / ROS 2 栈 | 构建、传感器、受控起降与闭环路线已运行 | [运行验收](RUNTIME_117.md) |
| 室内真实扫描绕障 | 部分指定路线通过，非整室任意点认证 | [扫描绕障](REAL_SCAN_AVOIDANCE_117.md) |
| 室外箱体/圆柱绕障 | 多条指定路线通过，重复性仍有失败 | [重复验证](OUTDOOR_REPEAT_EV_117.md) |
| 彩色 Baylands 外观 | 同参数短路线与 26.93 m 箱体绕障通过；各仅一轮 | [彩色复测](APPEARANCE_RETEST_117.md) |
| RViz 边缘目标准入 | 已修复界面允许而规划端点拒绝的检查差异；44 项回归通过，修复后未飞行复测 | [边缘航点诊断](RVIZ_EDGE_GOAL_117.md) |
| EV XY/Z 与共享边界 | 实际融合、部分长路线通过；yaw/速度不融合 | [共享边界与 EV](SHARED_ENVELOPE_EV_117.md) |
| 最新复杂/长距离路线 | 东侧 64.91 m 一轮完整通过；西侧精度失败；树旁超时；东侧重复拒绝 | [长距离漂移](OUTDOOR_LONG_DRIFT_117.md) |
| 长期/跨地图/实物 | 尚未通过 | [项目上下文](../../docs/PROJECT_CONTEXT.md) |

任务流程完成与完整认证分别记录。检查实际位移、测量配对、全曲线距离、动态约束、目标误差及恢复事件，不以退出码替代认证。

## 本次集成提交前检查

2026-10-14：完整 19 个 ROS 包构建通过；228 项 Python/独立 C++ 回归通过；FAST-LIVO2 的 5 项 CTest 和规划器 `tail_obstacle` 回归通过。当前使用文档的本地链接、修改代码语法和 `git diff --check` 通过。

此检查覆盖当前源码与文档，不新增飞行认证；构建仍有 PCL/CMake 既存提示，Python 有 protobuf 弃用提示。可复现命令见 [开发指南](../../docs/DEVELOPMENT.md)。

## 报告导航

| 范围 | 报告 |
| --- | --- |
| RViz 起飞后无横向轨迹与边缘目标检查修复 | [RVIZ_EDGE_GOAL_117.md](RVIZ_EDGE_GOAL_117.md) |
| 彩色场景的定位、延迟与闭环绕障复测 | [APPEARANCE_RETEST_117.md](APPEARANCE_RETEST_117.md)、[汇总 JSON](appearance_retest_117.json) |
| 最新室外复测与失败清单 | [OUTDOOR_REPEAT_EV_117.md](OUTDOOR_REPEAT_EV_117.md)、[汇总 JSON](outdoor_repeat_ev_117.json) |
| 最新分阶段/航段漂移与搜索失败 | [OUTDOOR_LONG_DRIFT_117.md](OUTDOOR_LONG_DRIFT_117.md)、[汇总 JSON](outdoor_long_drift_117.json) |
| 共享路线边界与 EV 接入 | [SHARED_ENVELOPE_EV_117.md](SHARED_ENVELOPE_EV_117.md) |
| 室外扩大范围的早期失败 | [OUTDOOR_EXPANSION_117.md](OUTDOOR_EXPANSION_117.md) |
| 箱体方向与圆柱等不同场景 | [OUTDOOR_GENERALIZATION_117.md](OUTDOOR_GENERALIZATION_117.md) |
| 重规划连续性与调度 | [OUTDOOR_CONTINUITY_117.md](OUTDOOR_CONTINUITY_117.md) |
| Baylands 场景和初始路线 | [OUTDOOR_PARK_117.md](OUTDOOR_PARK_117.md) |
| RViz 三维多航点与任务保护 | [RVIZ_GOAL_FLIGHT_117.md](RVIZ_GOAL_FLIGHT_117.md) |
| Replica office4 阻挡路径绕障 | [REAL_SCAN_AVOIDANCE_117.md](REAL_SCAN_AVOIDANCE_117.md) |
| 室内大范围与重复运行 | [LARGE_INDOOR_117.md](LARGE_INDOOR_117.md)、[INDOOR_TRAVERSAL_117.md](INDOOR_TRAVERSAL_117.md) |
| 扫描地图选择和首次接入 | [REAL_MAP_SELECTION.md](REAL_MAP_SELECTION.md)、[REPLICA_FLIGHT_117.md](REPLICA_FLIGHT_117.md) |
| odom 处理/传输/接收时间 | [ODOM_DELAY_117.md](ODOM_DELAY_117.md) |
| 基础迁移、传感器、起降 | [RUNTIME_117.md](RUNTIME_117.md) |

## 证据规则

- 原始 CSV、ULog、模型和视频保存在本机 `astra/runtime/<运行名>/`，不随仓库分发。汇总 JSON 为可审查结论，不能替代不存在的原始日志。
- 报告中的本机绝对路径是历史证据位置，当前复现使用 [操作指南](../../docs/OPERATIONS.md) 的相对路径命令。
- 不同修复版本的成功次数不构成同配置重复认证；候选曲线被拒绝不等同于该曲线已执行。
- 真值精度使用起飞前冻结参考；距离注明机体中心/已知表面，不能直接写成旋翼净空。
- 新报告须写日期、配置、运行名称、成功/失败、命令、指标和适用边界，并在本索引添加入口。
