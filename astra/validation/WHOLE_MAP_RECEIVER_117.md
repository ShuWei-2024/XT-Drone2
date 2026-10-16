# 全图远端往返：接收积压与事件执行器复测

2026-10-15，隔离 PX4 1.17 / Baylands 全图空域。前置范围与高度参考修正见 [全图空域报告](WHOLE_MAP_AIRSPACE_117.md)。本轮处理控制进程的接收开销，没有放宽定位、配对或轨迹保护。

## 故障证据与改动

原运行 `whole_map_dds100_east_20261015_01` 到达 `(27,10)` 后，在返航约 `(5.15,10.23)` 处中止。独立监视节点已经收到更新的 odom，执行桥仍在使用上一帧；桥时钟推进后测量年龄达到 252 ms，超过原 250 ms 门槛。故障前一次接收批次耗时约 106.9 ms，其中记录到的消息回调合计约 10.5 ms，其余发生在回调之前／之间。不能将全部耗时归为 FAST-LIVO2 算法计算。

已有可选 Jazzy EventsExecutor 使用事件队列接收，避免默认 Python 执行器反复构建／扫描等待集合。本轮用它复测同一远端目的地与返航，仍在同一个调用线程串行更新状态，保留每批 4 ms／最多 32 次追加调用、单一 setpoint 发布者、20 ms 配对、250 ms 测量年龄、0.20 m 高度差，以及原共享空域、净空与动态约束。

另增加只读 Python GC 暂停记录，不关闭垃圾回收、不改变阈值、不在飞行中强制回收。第一轮事件测试仍记录到约 69.8 ms 的二代 GC 停顿，但没有触发时效保护；旧失败运行没有 GC 记录，因此不能倒推其 106.9 ms 暂停全部由 GC 造成。

全图 `obstacle` 场景现在默认选择事件执行器，其他场景保留原默认；显式 `--execution-executor ready-batch` 可回退。运行器在启动仿真前检查安装的 Jazzy 是否提供 Python 与 C++ EventsExecutor，缺失则报错，不在起飞后才发现，也不静默回退。该 ROS API 仍属 experimental，仅作为已测 SITL 配置，不认证实物实时性。

## 远端重复结果

`whole_map_events_east_20261015_01`：完整审计通过。实际水平路径 68.974 m，完成东侧目的地和规划返航，安全降落上锁，无 armed failsafe。去程／返航终点真值 XY 误差 0.0731／0.0113 m；全程最小已知表面机体中心距离 1.2589 m。全部发布曲线共享空域、连续净空、速度与加速度检查通过，曲线峰值速度 0.20 m/s、加速度 0.2493 m/s²。视觉 XY/Z 实际融合。

| 指标 | 旧 ready-batch 失败轮 | 事件执行器第一轮 |
| --- | --- | --- |
| 远端往返 | 返航途中保护中止 | 完成并通过完整审计 |
| 实际水平路径 | 54.571 m，未完成 | 68.974 m，完成 |
| 接收批次尾段线程 CPU / 墙钟 | 55.6% | 25.8% |
| 桥 odom 回调相对独立监视节点的时间差 p95 / 最大 | 12.43 / 103.84 ms | 5.67 / 34.10 ms |

CPU 统计只覆盖各自保留尾段，不是整机占用或整轮平均；两轮不是同记录回放。独立节点回调时间差是传输／调度比较线索，可能为负，**不是 DDS 队列延迟**。当前安装的 EventsExecutor 不提供 MessageInfo，事件模式明确保留该证据缺口。第一轮虽完成，PX4 对 Gazebo XY 误差 p95 仍为 0.1135 m，超过 0.10 m 精密对齐门槛；功能完成不等于精密定位通过。

`whole_map_events_east_20261015_02`：不显式传执行器参数，运行记录确认自动选择 events；第二轮完整审计通过。实际水平路径 69.113 m，最小已知表面机体中心距离 1.4012 m，去程／返航终点真值 XY 误差 0.0801／0.0306 m。全部曲线共享空域、连续净空与动态约束通过，无 armed failsafe，降落上锁，原参数基线和执行源码保持不变。

| 指标 | 事件第一轮 | 事件第二轮 |
| --- | --- | --- |
| 监督器测量年龄 p95 / 最大 | 136 / 176 ms | 140 / 196 ms |
| 接收批次尾段线程 CPU / 墙钟 | 25.8% | 25.6% |
| 桥 odom 回调相对监视节点时间差 p95 / 最大 | 5.67 / 34.10 ms | 5.96 / 72.43 ms |
| 执行期冻结地面高度差 p95 / 最大 | 6.33 / 10.78 cm | 8.01 / 11.09 cm |
| FAST-LIVO2 真值 XY / Z 误差 p95 | 6.50 / 8.03 cm | 7.18 / 8.52 cm |
| PX4 真值 XY / Z 误差 p95 | 11.35 / 4.50 cm | 11.28 / 3.50 cm |

第二轮 GC 二代暂停最大约 70.9 ms，仍有短时接收等待。两轮都完成说明本次配置有改善，不证明所有超时已根治；PX4 XY 精密门槛两轮均未通过。第一轮显式指定事件模式，第二轮验证新的自动选择；期间只改自动选择／能力预检和 GC 结束时钟采样顺序，飞行控制、规划、算法、参数与全部保护限制保持相同，不能称为整个源码完全不变的重复认证。

## 地图掉线故障回归

`whole_map_events_mapstop_20261015_01` 在规划接管 2 秒后，只向本轮地图进程组发送 SIGINT。约 327.8 ms 后监督器报告 `local map not ready or heartbeat stale`，328.0 ms 后冻结保持目标；未完成任务，随后安全降落、上锁，`expected_fault_observed=true`，原参数与执行源码保持不变。该时间大于 250 ms，不能写成“250 ms 内完成全部故障反应”；测量年龄门槛与进程掉线至保持的总延迟是不同指标。

本轮官方 RTL 因 `PX4 failsafe flags stale` 被拒绝，在已检查下降区使用官方 Land。只证明这次掉线保护和受控下降生效，不认证本轮官方返航，也不覆盖下方无下降空间且无法 RTL 的故障。掉线任务的 `complete_route_verified=false` 被保留，故障回归通过不算正常飞行成功。

该拒绝有明确时间边界线索：PX4 `HealthAndArmingChecks.cpp` 在状态未变时仅在距离上次发布 **超过 500 ms** 后重发 failsafe flags（有状态变化则立即发布），本轮回调实测约每 540 ms 一次；监督器却要求接收年龄不超过 500 ms。没有状态中断也可能落入这段不满足门控的间隙。暂保留拒绝条件，后续需要在有界保持内等待新状态并重新检查全部返航条件，不能把旧状态直接判新或笼统放宽定位门槛。

## 西侧远端

`whole_map_events_west_20261015_01` 使用自动 events 默认，目的地 `(-27,0)`，完整审计通过。实际水平路径 54.574 m，去程／返航终点真值 XY 误差 0.0565／0.0389 m，最小已知表面机体中心距离 2.5622 m，监督器测量年龄 p95／最大 140／172 ms。曲线速度、加速度、连续净空和共享空域检查通过，实际 EV XY/Z 融合，无 armed failsafe，安全降落上锁。与东侧第二轮及地图故障轮相比，保存的全部执行源码／已列二进制哈希相同；区域和传感器观测不同。

本轮 FAST-LIVO2 对真值 XY/Z p95 为 3.57／7.52 cm，PX4 为 7.48／5.48 cm。这条西侧路线是较开阔的往返，不作为新的阻挡路径绕障证据，也不替代西侧同配置重复。三次正常任务全部通过是这三次的事实，不是所有地图或长期任务的成功率承诺。

## 复现

准备目录的方法见全图空域报告；每轮用新名称，一次一套仿真。第一轮显式选择 events，后续全图默认自动选择，实际值写入 `runtime.json` 和 `motion_events.json`。

```bash
bash astra/run_117.bash obstacle receiver_east_NEW \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_whole_map_east \
  --route-json astra/runtime/maps/baylands_whole_map_east/route.json \
  --calibrate-ground-height --external-vision --isolate-estimator-cpus \
  --executor-audit --end-action return_home

# 全部仿真退出后运行；不要在飞行中运行离线审计。
python3 astra/tools/audit_scan_flight.py astra/runtime/receiver_east_NEW
python3 astra/tools/audit_shared_envelope.py astra/runtime/receiver_east_NEW
python3 astra/tools/audit_external_vision.py astra/runtime/receiver_east_NEW
python3 astra/tools/audit_executor_timing_117.py astra/runtime/receiver_east_NEW \
  --output astra/runtime/receiver_east_NEW/executor_timing_analysis.json
```

262 项完整回归通过，包含同线程消息处理、原异常传播、时钟与计时器推进、接收预算、GC 诊断恢复和现有飞行门控。GC 计时使用包含线程 CPU 区间的墙钟区间，统计两个时钟时保留采样顺序。

本轮数值、基线恢复和执行源码差异见[证据摘要](whole_map_receiver_117_evidence.json)；原始运行资源在本机 `astra/runtime/`。跨区、独立地图和西侧重复仍待测试。本轮不认证长期内存、近障高速、坡地、风／动态障碍或实物。
