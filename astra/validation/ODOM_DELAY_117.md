# odom 接收延迟调查与可恢复修复（2026-10-11）

**后续扩大路线结果**：[LARGE_INDOOR_117.md](LARGE_INDOOR_117.md)。两轮
33 m 绕障及一次 8 m 横穿通过，但七段长路线再次触发一次 256 ms
测量年龄保护；接收开销改善尚未根治偶发超时。旧清理后接收年龄不可
直接当作飞行时延迟，已增加清理前故障时间快照。

延迟来源已缩小到 **LIO 线程调度压力 + Python 接收执行器开销**。本轮没有
改 FAST-LIVO2 算法、PX4 参数、传感器噪声或新鲜度门槛。新增的事件执行器
候选完成两轮约 29 m 飞行，未复现 odom 超时；**整体绕障重复安全仍未通过**，
因为第一轮发布轨迹的独立障碍距离审计失败。

完整数据：[executor_delay_117.json](executor_delay_117.json)。原始 ULog、
odom 测量时间、独立订阅监测、接收回调和源码快照保存在
`astra/runtime/<运行名称>/`。所有运行均正常降落、解除解锁并恢复原参数。

## 原故障证据

`indoor_hall_handoff_fix_117_02` 在仿真 254.568 s 停测，odom 测量年龄
268 ms、接收年龄约 281.7 ms，超过原 250 ms 限制。同一时刻其他订阅者
已经收到较新的位姿，不能把独立订阅者无掉流当作监督器收到及时。

对应 LIO 慢帧墙钟约 89.6 ms，其中线程 CPU 29.4 ms、运行队列等待
45.3 ms。因此不是“算法算了 89.6 ms”，也不能据此认定电脑无法运行。

## 接收程序问题和修改

原 `receive_batch` 先 `spin_once(.002)`，再反复 `spin_once(0)`。
空闲时仍反复扫描等待集合；切换等待参数还会使 rclpy 丢弃原 ready iterator，
重新扫描。监督器、preview、bridge 共用单线程执行器，约 250 Hz 的仿真
时钟和其他消息让扫描开销更明显。离线模拟输入的 cProfile 显示大量时间
在通信事件枚举、等待集合构建；其数值受 profiler 开销影响，不当作实飞指标。

已修改：

- 默认 `ready-batch` 保持一致的等待参数，遇到空闲等待立即结束一批；
  原有 4 ms 处理预算和最多 32 次追加调用保留。
- 新增 `--execution-executor events`，使用当前 Jazzy 提供的实验性 C++
  EventsExecutor。仍由调用线程串行执行回调，每批同样有时间和次数上限，
  没有引入多个线程共享飞行状态。
- `--executor-audit` 可记录回调墙钟/线程 CPU、测量时间、独立消息到达和
  批处理调度等待。跟踪数据只在降落解除解锁后写盘，避免审计自身阻塞飞行。
- EventsExecutor 在本机 rclpy `7.1.12-1noble.20260902.053513` 不支持
  MessageInfo 回调。诊断模式对此明确记录缺失，使用原消息回调；**没有把
  缺失的 DDS 时间填成有效零延迟**。事件模式的 DDS 队列延迟不能认证。

默认仍为 `ready-batch`，事件候选用显式参数启用，便于回退。PX4/LIVO
接收和测量时间 250 ms 门槛、未来时间戳保护、障碍/高度保护及 setpoint
单一发布者规则保留。飞行期间未编译、绘图或运行离线分析。

## 实测结果

| 运行 | 执行/保护结果 | 接收批处理 CPU 占一核比例 | 执行中 odom 测量年龄 p95 / 最大 |
| --- | --- | ---: | ---: |
| indoor_executor_trace_117_02 | 旧执行器，首段高度差 >0.20 m 停测 | 约 85% | 158.6 / 204 ms |
| indoor_ready_batch_117_01 | 默认有界接收，返回段高度差停测 | 约 81% | 144 / 216 ms |
| indoor_events_executor_117_01 | 四段完成、降落，规划距离审计失败 | 约 30% | 136 / 192 ms |
| indoor_events_executor_117_02 | 四段完整严格验收通过、降落 | 约 30% | 136 / 212 ms |

CPU 比例是诊断尾段中的 **接收批处理线程 CPU / 尾段墙钟跨度**，不是
整台电脑占用，也不是整轮飞行平均。各轮输入和噪声不同，第一条执行期只有
约 6.8 s；这些不是相同记录的成对回放，不能把全部差异归因于执行器。

有界 Python 接收的监督器 DDS 到回调等待 p95 从约 10.4 ms 降到 4.7 ms，
最大从约 43.4 ms 降到 11.9 ms，但 CPU 降幅有限。事件候选避免了主要
Python 扫描开销；两轮同一 odom 在三个节点间的回调时间跨度 p95 约
1.50 / 1.53 ms，最大 11.82 / 4.74 ms，没有节点出现 >250 ms 接收间隔。

原故障运行的 LIO 墙钟 p95 44.4 ms，事件候选为 38.2 / 38.8 ms，
调度等待 p95 从约 9.2 ms 降到 6.0 / 5.9 ms。偶发墙钟最大仍达到
105.2 / 91.7 ms，CPU 最大只有约 35 ms；**慢帧和主机竞争没有被证明彻底消失**。

执行时的测量年龄还包含 10 Hz odom 两次更新之间的等待，不能把
136 ms 全部解释为一次 LIO 计算耗时。它与“DDS 到回调等待”是不同指标。

两轮事件候选的实际水平航程 29.243 / 29.560 m，全程已知障碍最小距离
0.756 / 0.783 m，合计 setpoint 最大发送间隔约 62.8 ms（第一轮）。
执行期均无 XY/Z/yaw reset，无 armed failsafe，External Vision 融合关闭。

第一轮 233 条原始发布轨迹中，8 条的独立连续距离保守下界低于 0.55 m，
最小约 0.4526 m；实际飞行距离虽然通过，仍保留
`complete_route_verified=false`，不能算整体重复安全通过。第二轮规划和
实际距离、四段端点及交接均通过。后续需区分这些不足轨迹是否执行、被替换
或撤销，并修复规划距离问题；本轮没有改审计规则隐藏失败。

## 故障保护回归

`indoor_events_mapstop_117_01` 在轨迹接管 2 s 后，仅停止当前运行拥有的
地图进程组。停止命令后约 304.8 ms 报告地图/heartbeat 失效，305.2 ms
冻结 PX4 保持目标，随后降落解除解锁。`fault_regression.json` 为通过。
这是原有地图超时与控制循环的实测反应，不是 250 ms 内失效认证；odom 的
250 ms 门槛未改变。

另一次 `indoor_executor_trace_117_01` 的初版审计在空中序列化约 8.7 MB
JSON，导致人为约 384 ms 暂停和终点保护失败。已修为解除解锁后保存，
该失败不作为定位算法延迟证据，也不计为成功飞行。

## 运行、审计与回退

每轮使用新名称；保持所有模拟器退出后再做离线分析。

```bash
cd /home/cyf/dev/XT-Drone2-astra
# 从干净终端运行事件候选，明确仅用于隔离 SITL
bash astra/run_117.bash obstacle hall_events_next \
 --world-preset indoor-hall --calibrate-ground-height \
 --route-json astra/config/route_indoor_hall_traverse.json \
 --plane-quality-diagnostic --executor-audit --execution-executor events

# 回退至默认有界 Python 执行器：替换参数为
# --execution-executor ready-batch

# 地图掉线回归：在飞行命令末尾追加
# --stop-map-after-handoff-s 2

# 仿真全部退出后
python3 astra/tools/audit_executor_timing_117.py astra/runtime/hall_events_next \
 --output astra/validation/hall_events_next_timing.json
python3 astra/tools/audit_odom_delivery_117.py astra/runtime/hall_events_next \
 --output astra/validation/hall_events_next_delivery.json
python3 astra/tools/audit_indoor_hall.py astra/runtime/hall_events_next
```

本轮实际启动命令均包在
`env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONPATH
-u LD_LIBRARY_PATH bash --noprofile --norc -c '上述启动命令'` 中。

61 项相关回归通过：消息跨节点接收、同一调用线程执行、模拟时钟与 steady
timer、原异常传播、DDS 元数据缺失处理、批处理上限、坐标/高度门控、轨迹
交接和执行桥保护。原 PX4 1.15 参数/源码哈希和每轮执行源码/二进制哈希均
核对保持不变。未提交或推送 Git。
