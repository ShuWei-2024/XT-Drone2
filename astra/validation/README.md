# 验证索引与当前状态

[总文档索引](../../docs/README.md) · [运行与审计方法](../../docs/OPERATIONS.md)

更新：2026-10-22。报告记录当时版本与场景，历史失败不删除；先阅读三项失败分解、Baylands 固定配置复测和长时运动审计，再按需查历史专题。

最新离线逐项检查：第 6 轮实际 Odom 发布调用间隔为 `251.543 ms`，旧 `257.164 ms` 是处理结束代理；适配器 IMU 回调缺口约 131 ms 时，独立监视器仍收到 PX4 和 readiness，执行进程没有同期 GC，适配器自身根因仍缺证据。第 3 轮冻结高度残差 `−0.2013 m` 可分解为 LIVO 相对地面真值 `−0.1551 m` 与 PX4 `+0.0462 m`。第 4 轮 ULog 显示初始化后位置变化 `0.0300788 m`，保持原门槛；新增精确失败样本记录和离线工具，25 项针对性回归通过，没有新增飞行。见[三项失败逐项检查](ACCEPTANCE_FAILURES_20261022.md)。

最新多位置恢复：远端约 10 m 完整 DDS 断链、障碍旁地图停止、不可下降区地图停止三项有效试验均通过；检测约 209／226／237 ms，分别自主 RTL、原地降落、撤至可下降区降落，ULog 确认上锁。原保护保持，四轮未通过记录保留；最终 392 项 Python 回归通过。详见[多位置故障恢复](MULTILOCATION_FAULTS_117.md)。

最新故障验收：起点附近可下降区的真实 XY、含高度 XY/Z、航向重置和完整 DDS Agent 退出四项均通过。执行端检测约 36.65/21.38/26.53/211.06 ms，检测后无正常轨迹采样，机载 ULog 确认各轮落地上锁；断链轮 PX4 约 1.04 s 后自行进入 Land，ROS 不伪造上锁确认。原保护门槛、PX4 参数和二进制保留。最终 382 项 Python 回归通过。保留三轮无有效注入的测试入口/EV 超时记录；不认证所有位置、所有链路或超时永久消失。详见[重置与完整 DDS 断链验收](ESTIMATOR_LINK_FAULTS_117.md)。

最新 Baylands 固定配置收尾：`acceptance_baylands_20261021_01/02/03/05` 使用同一 129 项执行源码 manifest；4 次真正进入飞行中 3 次完成约 71 m 三航点、EGO 返航、300 s 保持和降落上锁，1 次由既有高度差 `0.20 m` 保护中止后上锁。`..._04` 在解锁前因原点初始化位置变化超过 `0.03 m` 退出，不计为飞行失败。第 5 轮实测路径 `71.238 m`、最小已知表面机体中心净空 `1.261 m`、配对 p95/max `8/12 ms`，EV armed XY/Z 融合比例均为 100%。该组支持指定路线流程重复通过；高度漂移根因、详细审计内存和跨地图适用性仍未通过。详见[Baylands 固定配置收尾复测](BAYLANDS_ACCEPTANCE_20261021.md)。

第 6 轮同一 129 项配置的 1500 s 连续运动目标在仿真 `487.580 s` 因 FAST-LIVO2 Odom 测量年龄 `268 ms` 触发既有保护，实际运动 `362.886 s`、`55.925 m`、到达 2/3 航点，随后受控降落上锁；PX4 ULog 没有 armed failsafe。输入审计显示相关同步回调缺口约 `131.1 ms`，这轮不计入正常路线成功次数。详见[Baylands 长时运动中止审计](BAYLANDS_LONG_MOTION_20261021.md)。

最新正常配置验收：`normal_long_acceptance_20261020_01/02/03` 使用相同 126 项源码/二进制哈希、估计器 YAML、PX4 覆盖参数与 CPU 分配，正常图像融合三轮均完成约 71 m 绕障、返航、300 s 保持及降落上锁。监督器测量年龄最大 204/204/212 ms，接收年龄最大 166.57/151.90/173.90 ms；250 ms 等原门槛保持。初始化后 25631 次 LIO 全部发布 Odom，EV XY/Z 实际融合，无 armed failsafe。执行进程保持增长仅 0.070/0.055/0.074 MiB，末 120 s 均不增长；LIVO 首轮尾段仍增长。LIVO 运动 XY p95 为 0.1089/0.0776/0.1286 m，不能认定全程小于 0.10 m。**本配置指定路线与时效重复通过，历史高度失败的根因仍未证明修复。** 详见[正常融合长路线重复验收](NORMAL_LONG_ACCEPTANCE_117.md)。

先前时序调查：两轮 `−12 ms` 图像偏移试验中，原始 LiDAR 已到达，部分融合切片却早于本帧采样。新静态对照确认：0 偏移初始化后 568 次全部发布；−12 ms 出现 18 次零点切片，处理只需 135～220 µs。不能据此认定传感器断流、Python 暂停或算力不足。默认仍为 0，保护保持；核心重建、5/5 CTest、368 项 Python 回归通过。另外严格拒绝 VIO 修正的长路线仍有高度失败，不能把大幅视觉修正当成已证实根因。详见[测量切片时间审计](LIVO_CUT_TIMING_117.md)。

先前对照：详细审计 Odom 历史改为等容量预分配存储，四短航点＋300 s 保持通过，执行进程保持 RSS 增 0.637 MiB、最后 120 s 增 0.176 MiB。完整融合长路线重复在实际 33.920 m 后因高度差超过 0.20 m 中止并上锁；关闭视觉更新的对照完成 71.008 m 绕障与返航，LIVO 真值 Z p95 从失败轮 0.1650 m 到诊断轮 0.0244 m。两轮初始化/源码阶段不同，**当时完整融合长路线重复未通过**。出发报告有界等待修复后，完整回归 344 项通过；该飞行未触发等待分支。详见[审计缓存与高度复测](COMPACT_AUDIT_HEIGHT_117.md)。

先前增量记录版本完成 71.301 m 双航点＋EGO 返航＋300 s 保持，419 条曲线审计通过，端点误差 1.94～6.20 cm，最小机体中心净空 1.445 m，正常上锁，无 armed failsafe。EV XY/Z 实际融合，合并发送记录最大间隔 71.28 ms。随后三项独立故障子项（定位算法、规划器、执行端 PX4 位置输入）均在约 231～286 ms 内冻结／锁存并最终上锁；旧整 launch 联合中断不计入纯定位故障。该阶段 338 项回归通过。详见[收尾验收](SIMULATION_ACCEPTANCE_117.md)和[日志证据](GC_RECEIVER_117.md)。

内存须分配置判断：关闭详细审计的旧 180 s 保持执行端 +0.0469 MiB；预分配前开启审计的长路线 300 s 保持增 6.59 MiB，预分配后短路线保持增 0.637 MiB。路线历史不同，不作严格 A/B 或无限稳定证明，也不把不同源码阶段混算为同版成功率。

2026-10-18 延长验收已完成原双航点、EGO 返航、5 分钟空中保持与降落：真值路径 71.205 m，419 条曲线检查通过，无 armed failsafe。全窗口 odom 间隔最大 178.1 ms、setpoint 76.8 ms；保持阶段出现一次 45.54 ms 二代 GC，未触发保护。LIVO 后段接近平台，但执行子进程保持阶段仍增加 13.5 MiB，不能称全栈长期稳定。地图和 EV 进程分别中断后的恢复均通过一次。详见[收尾验收](SIMULATION_ACCEPTANCE_117.md)与[GC 接收复测](GC_RECEIVER_117.md)。

## 当前状态

| 项目 | 结论 | 主要证据 |
| --- | --- | --- |
| 多位置故障恢复 | 远端 DDS 失联自主 RTL、障碍旁地图失效降落、不可下降区地图失效撤退均通过各一次；不是任意位置断链认证 | [多位置恢复](MULTILOCATION_FAULTS_117.md) |
| 空中重置与完整 DDS 断链 | XY、含高度、航向重置及 Agent 失联四项有效注入通过；机载确认落地上锁，限定起点附近 | [最新故障验收](ESTIMATOR_LINK_FAULTS_117.md) |
| Baylands 固定配置收尾 | 129 项固定 manifest；进入飞行 4 次，3 次完整完成、1 次高度保护中止；另 1 次未解锁门控 | [固定配置收尾](BAYLANDS_ACCEPTANCE_20261021.md) |
| Baylands 长时运动 | 1500 s 目标在 362.886 s 运动后由 250 ms Odom 门槛保护中止；无 armed failsafe | [长时运动中止审计](BAYLANDS_LONG_MOTION_20261021.md) |
| 正常融合长路线重复 | 同配置三轮约 71 m＋300 s 保持均通过，原年龄门槛内；非全程 10 cm 精度或实物认证 | [最新重复验收](NORMAL_LONG_ACCEPTANCE_117.md) |
| 审计缓存与长路线对照 | 短路线五分钟保持通过；完整融合长路线触发高度保护，LIO-only 71.008 m 诊断通过；保留当时完整融合重复失败 | [最新收尾复测](COMPACT_AUDIT_HEIGHT_117.md) |
| 仿真收尾验收 | 已有完整融合长路线和三项独立故障子项通过；同配置正常路线三轮通过；起点附近重置/完整 DDS 失联已补齐；多位置恢复与扩大运行内存仍待补齐 | [收尾验收](SIMULATION_ACCEPTANCE_117.md) |
| 连通空域开放与恢复 | ±34 m、约 3376 m²；新增通行点地图失联退出一次通过，西侧 31 m 正常任务与附近降落一次通过；299 项回归通过，全部点未逐点试飞 | [连通空域](CONNECTED_AIRSPACE_117.md) |
| PX4 1.17 / ROS 2 栈 | 构建、传感器、受控起降与闭环路线已运行 | [运行验收](RUNTIME_117.md) |
| 室内真实扫描绕障 | 部分指定路线通过，非整室任意点认证 | [扫描绕障](REAL_SCAN_AVOIDANCE_117.md) |
| 室外箱体/圆柱绕障 | 多条指定路线通过，重复性仍有失败 | [重复验证](OUTDOOR_REPEAT_EV_117.md) |
| 彩色 Baylands 外观 | 同参数短路线与 26.93 m 箱体绕障通过；各仅一轮 | [彩色复测](APPEARANCE_RETEST_117.md) |
| RViz 边缘目标准入 | 已修复界面允许而规划端点拒绝的检查差异；44 项回归通过，修复后未飞行复测 | [边缘航点诊断](RVIZ_EDGE_GOAL_117.md) |
| 通行／降落分离与返航 | 合并区域往返、地图中断后官方 RTL 各通过；高度精度仍有限制，240 项回归通过 | [通行与返航](TRANSIT_RETURN_117.md) |
| 全图范围选点 | ±30 m 边界、1265 m² 可选面积；事件模式东侧两轮约 69 m、西侧一轮 54.6 m 往返通过；地图掉线受控降落通过，仍有精度／恢复限制 | [接收积压复测](WHOLE_MAP_RECEIVER_117.md)、[全图空域](WHOLE_MAP_AIRSPACE_117.md) |
| 接收暂停与记录内存修复 | 338 项回归通过；长路线原门槛通过，合并发送最大 71.28 ms；关闭审计的短保持近平台，详细审计开启仍增长 | [GC 接收复测](GC_RECEIVER_117.md) |
| EV／全局原点启动确认 | 289 项回归通过；地面复测抓到 GNSS 超前 4 ms，重新连续确认后进入选点；未飞行 | [原点初始化](EV_ORIGIN_STARTUP_117.md) |
| 航点范围可视化／几何精化 | 70 m 参考网格、青色静态端点范围；同余量增 34.64 m²，一个新增航点 7.99 m 往返通过一次 | [范围与扩区](GOAL_REGION_REFINEMENT_117.md) |
| RViz 原双航点＋返航 | 70.84 m 一轮完整审计通过；420 条曲线、连续净空与动态约束通过；保留旧曲线分支只有回归证据 | [重规划与交接修复](REPLAN_BOUNDARY_117.md) |
| EV XY/Z 与共享边界 | 实际融合、部分长路线通过；yaw/速度不融合 | [共享边界与 EV](SHARED_ENVELOPE_EV_117.md) |
| 早期复杂/长距离路线 | 东侧 64.91 m 一轮完整通过；曾有西侧精度失败、树旁超时、东侧重复拒绝 | [长距离漂移](OUTDOOR_LONG_DRIFT_117.md) |
| 长期/跨地图/实物 | 尚未通过 | [项目上下文](../../docs/PROJECT_CONTEXT.md) |

任务流程完成与完整认证分别记录。检查实际位移、测量配对、全曲线距离、动态约束、目标误差及恢复事件，不以退出码替代认证。

## 本次集成提交前检查

2026-10-20（多位置恢复）：三项有效试验通过并独立确认机载落地上锁；新增到点故障触发、悬停时间锚点、只读机载观察和缺失内存数组的离线执行证据读取。最终 **392 项 Python 回归通过**，保留 3 条既有弃用提示；语法、链接和差异空白检查通过。原 PX4 参数和二进制保留，各轮源码在飞行期间不变。

2026-10-20（故障验收）：隔离运行器新增真实 reset 命令、地面预建注入器、完整 DDS Agent 中断及闭合 ULog 的独立验收。有效 XY、含高度、航向和 DDS 故障四轮通过；三轮无有效注入和中途 EV 超时不计入通过。最终 382 项 Python 回归、语法及文档链接检查通过；保留 3 条既有弃用提示。

2026-10-19（缓存与出发门控）：紧凑审计记录和周期报告等待修复后，最新完整 Python 回归 344 项通过，保留 3 条既有 protobuf 弃用提示。短路线五分钟保持通过，完整融合长路线失败及关闭图像的长路线诊断结果均保留；不合并为同版本重复认证。

2026-10-19（故障工具收尾）：故障注入目标核验及失败处理调整后，语法检查、338 项完整 Python 回归、15 项故障／GC 专项通过；保留 3 条既有 protobuf 弃用提示。补齐长路线和独立故障矩阵的直接入口。此检查未新增 SITL 飞行，不增加同版重复次数。

2026-10-16（原点启动）：289 项回归通过；地面复测实际恢复一次临时超前 4 ms 的 GNSS 测量，重新连续健康确认后进入选点，未解锁或飞行。原 60 秒总期限、1 秒连续确认及各年龄／位移门槛保持。

2026-10-16（扩区）：283 项回归通过；3485 个新增下降柱体连续净空逐格审计通过，原可选格无减少。新增目标 `(2.45,-2.95)` 一轮 7.99 m 往返完整审计通过，40 条曲线均合格；本轮不是阻挡路径绕障试验，不认证全部新增区域。

2026-10-16：280 项完整回归通过；原用户双航点＋规划返航 70.84 m 一轮完整审计通过，最小已知表面机体中心距离 1.209 m，无 armed failsafe。修复重规划失败衔接、同进程预览交接顺序及共享控制时钟；调试失败、调度余留风险与覆盖边界见 [重规划报告](REPLAN_BOUNDARY_117.md)。

2026-10-16：277 项完整回归通过；最终配置 RViz 地面等待、起飞、22.37 m 已知表面绕障往返完整审计通过。调试失败和测试边界见交互起飞报告；不等于用户原两个航点或任意全图路线已验证。

2026-10-15：262 项完整回归通过；事件模式东侧两轮、西侧一轮正常飞行完整审计通过，另有一轮地图掉线保护回归通过。各轮原 PX4 参数基线与执行源码保持不变；两轮东侧之间的 runner／诊断差异及精度限制见接收积压报告。没有因任务流程完成而认证跨地图或实物。

2026-10-14：完整 19 个 ROS 包构建通过；228 项 Python/独立 C++ 回归通过；FAST-LIVO2 的 5 项 CTest 和规划器 `tail_obstacle` 回归通过。当前使用文档的本地链接、修改代码语法和 `git diff --check` 通过。

此检查覆盖当前源码与文档，不新增飞行认证；构建仍有 PCL/CMake 既存提示，Python 有 protobuf 弃用提示。可复现命令见 [开发指南](../../docs/DEVELOPMENT.md)。

## 报告导航

| 范围 | 报告 |
| --- | --- |
| Baylands 延迟尖峰、高度累计偏差与原点初始化拒绝 | [三项失败逐项检查](ACCEPTANCE_FAILURES_20261022.md)、[证据 JSON](acceptance_failures_20261022.json) |
| 空中真实重置与全部 PX4 DDS 传输中断 | [ESTIMATOR_LINK_FAULTS_117.md](ESTIMATOR_LINK_FAULTS_117.md)、[汇总 JSON](estimator_link_faults_117.json) |
| 正常融合三轮长路线、五分钟保持、延迟与内存 | [NORMAL_LONG_ACCEPTANCE_117.md](NORMAL_LONG_ACCEPTANCE_117.md)、[汇总 JSON](normal_long_acceptance_117.json) |
| 图像偏移敏感性、空切片与实际发布时刻 | [LIVO_CUT_TIMING_117.md](LIVO_CUT_TIMING_117.md)、[汇总 JSON](livo_cut_timing_117.json) |
| 预分配审计缓存、五分钟保持、完整融合失败与 LIO 对照 | [COMPACT_AUDIT_HEIGHT_117.md](COMPACT_AUDIT_HEIGHT_117.md)、[汇总 JSON](compact_audit_117.json) |
| 仿真收尾范围、失效矩阵与硬件前边界 | [SIMULATION_ACCEPTANCE_117.md](SIMULATION_ACCEPTANCE_117.md) |
| Python GC 暂停、接收历史与地图／EV 失联复测 | [GC_RECEIVER_117.md](GC_RECEIVER_117.md) |
| 先前完整融合 71.301 m、EV 融合与 300 s 保持 | [长路线证据 JSON](final_long_ev_117.json)、[内存曲线](final_long_memory_117.png) |
| 定位算法／规划器／执行端位置输入独立故障 | [故障矩阵 JSON](final_fault_matrix_117.json) |
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
