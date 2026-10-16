# 验证索引与当前状态

[总文档索引](../../docs/README.md) · [运行与审计方法](../../docs/OPERATIONS.md)

更新：2026-10-16。报告记录当时版本与场景，历史失败不删除；判断当前状态先阅读 RViz 重规划衔接、起飞前接收修复、全图接收积压复测与空域报告，再按需阅读早期专题。

## 当前状态

| 项目 | 结论 | 主要证据 |
| --- | --- | --- |
| PX4 1.17 / ROS 2 栈 | 构建、传感器、受控起降与闭环路线已运行 | [运行验收](RUNTIME_117.md) |
| 室内真实扫描绕障 | 部分指定路线通过，非整室任意点认证 | [扫描绕障](REAL_SCAN_AVOIDANCE_117.md) |
| 室外箱体/圆柱绕障 | 多条指定路线通过，重复性仍有失败 | [重复验证](OUTDOOR_REPEAT_EV_117.md) |
| 彩色 Baylands 外观 | 同参数短路线与 26.93 m 箱体绕障通过；各仅一轮 | [彩色复测](APPEARANCE_RETEST_117.md) |
| RViz 边缘目标准入 | 已修复界面允许而规划端点拒绝的检查差异；44 项回归通过，修复后未飞行复测 | [边缘航点诊断](RVIZ_EDGE_GOAL_117.md) |
| 通行／降落分离与返航 | 合并区域往返、地图中断后官方 RTL 各通过；高度精度仍有限制，240 项回归通过 | [通行与返航](TRANSIT_RETURN_117.md) |
| 全图范围选点 | ±30 m 边界、1265 m² 可选面积；事件模式东侧两轮约 69 m、西侧一轮 54.6 m 往返通过；地图掉线受控降落通过，仍有精度／恢复限制 | [接收积压复测](WHOLE_MAP_RECEIVER_117.md)、[全图空域](WHOLE_MAP_AIRSPACE_117.md) |
| 航点范围可视化／几何精化 | 70 m 参考网格、青色静态端点范围；同余量增 34.64 m²，一个新增航点 7.99 m 往返通过一次 | [范围与扩区](GOAL_REGION_REFINEMENT_117.md) |
| RViz 原双航点＋返航 | 70.84 m 一轮完整审计通过；420 条曲线、连续净空与动态约束通过；保留旧曲线分支只有回归证据 | [重规划与交接修复](REPLAN_BOUNDARY_117.md) |
| EV XY/Z 与共享边界 | 实际融合、部分长路线通过；yaw/速度不融合 | [共享边界与 EV](SHARED_ENVELOPE_EV_117.md) |
| 早期复杂/长距离路线 | 东侧 64.91 m 一轮完整通过；曾有西侧精度失败、树旁超时、东侧重复拒绝 | [长距离漂移](OUTDOOR_LONG_DRIFT_117.md) |
| 长期/跨地图/实物 | 尚未通过 | [项目上下文](../../docs/PROJECT_CONTEXT.md) |

任务流程完成与完整认证分别记录。检查实际位移、测量配对、全曲线距离、动态约束、目标误差及恢复事件，不以退出码替代认证。

## 本次集成提交前检查

2026-10-16（扩区）：283 项回归通过；3485 个新增下降柱体连续净空逐格审计通过，原可选格无减少。新增目标 `(2.45,-2.95)` 一轮 7.99 m 往返完整审计通过，40 条曲线均合格；本轮不是阻挡路径绕障试验，不认证全部新增区域。

2026-10-16（最新）：280 项完整回归通过；原用户双航点＋规划返航 70.84 m 一轮完整审计通过，最小已知表面机体中心距离 1.209 m，无 armed failsafe。修复重规划失败衔接、同进程预览交接顺序及共享控制时钟；调试失败、调度余留风险与覆盖边界见 [重规划报告](REPLAN_BOUNDARY_117.md)。

2026-10-16：277 项完整回归通过；最终配置 RViz 地面等待、起飞、22.37 m 已知表面绕障往返完整审计通过。调试失败和测试边界见交互起飞报告；不等于用户原两个航点或任意全图路线已验证。

2026-10-15：262 项完整回归通过；事件模式东侧两轮、西侧一轮正常飞行完整审计通过，另有一轮地图掉线保护回归通过。各轮原 PX4 参数基线与执行源码保持不变；两轮东侧之间的 runner／诊断差异及精度限制见接收积压报告。没有因任务流程完成而认证跨地图或实物。

2026-10-14：完整 19 个 ROS 包构建通过；228 项 Python/独立 C++ 回归通过；FAST-LIVO2 的 5 项 CTest 和规划器 `tail_obstacle` 回归通过。当前使用文档的本地链接、修改代码语法和 `git diff --check` 通过。

此检查覆盖当前源码与文档，不新增飞行认证；构建仍有 PCL/CMake 既存提示，Python 有 protobuf 弃用提示。可复现命令见 [开发指南](../../docs/DEVELOPMENT.md)。

## 报告导航

| 范围 | 报告 |
| --- | --- |
| RViz 转弯重规划失败、保留已检查旧曲线与有界重试 | [REPLAN_BOUNDARY_117.md](REPLAN_BOUNDARY_117.md) |
| RViz 起飞前消息接收、显示缓存与任务生命周期 | [RVIZ_PREFLIGHT_RECEIVER_117.md](RVIZ_PREFLIGHT_RECEIVER_117.md) |
| 全图远端往返、接收积压与事件执行器 | [WHOLE_MAP_RECEIVER_117.md](WHOLE_MAP_RECEIVER_117.md) |
| 独立巡航／降落区、规划返航与官方 RTL | [TRANSIT_RETURN_117.md](TRANSIT_RETURN_117.md) |
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
