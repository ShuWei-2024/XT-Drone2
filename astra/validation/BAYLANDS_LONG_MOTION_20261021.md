# Baylands 长时运动中止审计

日期：2026-10-21。运行目录：`astra/runtime/acceptance_baylands_20261021_06`。本轮沿用 PX4 1.17、ROS 2 Jazzy、Gazebo Baylands、FAST-LIVO2 正常图像融合、External Vision XY/Z 融合和事件执行器；没有改变 Odom 年龄、测量配对或高度保护门槛。

## 运行目标与结果

目标是连续运动 1500 s、重复路线 10 次。实际在仿真 `487.580 s` 因既有 `FAST-LIVO2 measurement age outside 0..250 ms` 保护中止，随后受控降落并上锁；没有触发 PX4 failsafe。

| 项目 | 实测结果 |
| --- | ---: |
| 有效运动时间 | 362.886 s |
| 运动距离 | 55.925 m |
| 测量时间真值水平路径 | 56.416 m |
| 峰值三维速度 | 0.242 m/s |
| 已到达航点 | 2/3 |
| 中止时 Odom 测量年龄 | 268 ms |
| 中止时 Odom 接收年龄 | 243 ms |
| 已知表面机体中心最小净空 | 1.263 m |
| 共享空域越界 | 0 个样本 |
| armed PX4 failsafe | 0 个样本 |

前两个航点的真值端点误差为 `3.94 cm` 和 `1.53 cm`；第三个航段在中止前只完成部分回程，不能算完成。规划曲线的连续净空、速度/加速度约束、目标交接保持和合并 setpoint 间隔均通过离线检查；这不等于 1500 s 长时任务通过。

## Odom 超时的时间证据

中止时最后一个已完成 Odom 测量时间为 `487.312 s`，下一帧测量时间为 `487.412 s`。下一帧最终确实发布，但发布时间为 `487.588 s`，在保护触发（`487.580 s`）之后约 8 ms 才被独立监视器收到。因此这次是保护在“下一帧即将到达但尚未完成发布”时生效，不是 Odom 永久停止。

离线数据给出的分解如下：

- LIO 处理结束记录间隔最大 `257.164 ms`（发布节奏的代理值）；利用实际发布调用结束字段核对为 `251.543 ms`，详见[后续逐项检查](ACCEPTANCE_FAILURES_20261022.md)；独立监视器回调间隔最大 `249.418 ms`；
- 下一帧 LIO 墙钟处理耗时 `56.324 ms`，其中线程 CPU `18.473 ms`、调度等待 `34.739 ms`；
- LIO 的处理 p95 为 `29.305 ms`，所以没有证据表明每帧计算都超过 250 ms；
- 输入审计在相关 LiDAR/IMU 缓冲后出现约 `131.1 ms` 的 IMU 回调缺口（同步定时回调缺口约 `149.1 ms`），且适配器记录了 LiDAR/RGB `88 ms`、LiDAR/IMU `80 ms` 的时间差告警；
- LIO 队列丢点为零，首帧发布后没有空切片或无输出帧；当前 EventsExecutor 没有 DDS MessageInfo，不能把剩余时间精确归因到 DDS 队列。

所以当前最可信的结论是：**这次超时由传感器同步/适配器回调缺口叠加 LIO 调度等待形成；不能仅凭本轮日志断言是 Python GC、DDS 或单纯 LIO 算力。** 保护门槛保持 250 ms，不用放宽门槛来把本轮改写成成功。

PX4 ULog 显示：`488.504 s` 开始执行当前点降落，`494.088 s` 因降落上锁；没有 `failsafe=true` 的 armed 样本，也没有新的估计器 reset 作为中止原因。

## 轨迹、External Vision 与内存

- 轨迹实际只完成两个航点；全程测量时间净空最低 `1.2626 m`，共享边界没有越界；
- PX4 External Vision 位置和高度在 armed 样本中均实际融合，比例为 `100%`；本轮不融合视觉 yaw/速度；
- LIVO RSS 从解锁后约 `203.6 MiB` 增至中止前约 `468.6 MiB`，最后 120 s 仍增加约 `1.9 MiB`；保留图像达到上限 `513` 张、约 `150.3 MiB`；
- 控制/审计进程在飞行窗口内增加约 `43.1 MiB`。落地后的导出阶段短暂峰值约 `236.2 MiB`，不能把这个落地后峰值当作空中增长；
- 这些结果仍不能证明扩大地图或无限运行的内存稳定性。

## 复现与审计命令

原始 shell 历史未随运行归档；以下启动命令依据 `runtime.json` 中保存的配置重建，使用新运行名重跑时会创建新的隔离 SITL 运行并执行运动任务：

```bash
bash astra/run_117.bash obstacle acceptance_baylands_long_repro \
  --world-preset baylands \
  --scan-dir astra/runtime/maps/baylands_gc_receiver_20261018 \
  --route-json astra/runtime/maps/baylands_gc_receiver_20261018/route.json \
  --calibrate-ground-height --external-vision --executor-audit \
  --isolate-estimator-cpus --execution-executor events \
  --end-action return_home --endpoint-hold-seconds 300 \
  --continuous-motion-seconds 1500 --route-repeat-cycles 10
```

运行目录的 `runtime.json`、冻结的 `runner.py`、`execution_sources/manifest.json` 和参数备份可用于核对配置与源码。仿真停止后可用以下离线审计复核：

```bash
python3 astra/tools/audit_odom_delivery_117.py \
  astra/runtime/acceptance_baylands_20261021_06 \
  --output astra/runtime/acceptance_baylands_20261021_06/odom_delivery_audit.json

python3 astra/tools/audit_executor_timing_117.py \
  astra/runtime/acceptance_baylands_20261021_06 \
  --output astra/runtime/acceptance_baylands_20261021_06/executor_timing_audit.json

python3 astra/tools/audit_scan_flight.py \
  astra/runtime/acceptance_baylands_20261021_06

python3 astra/tools/audit_runtime_growth_117.py \
  astra/runtime/acceptance_baylands_20261021_06 \
  --output astra/runtime/acceptance_baylands_20261021_06/runtime_growth_audit.json
```

## 判定

本轮分类为：**长时运动未通过，保护动作通过**。它补充了长时间尾部延迟和空中内存窗口的证据，但不能增加正常路线成功次数，也不能证明 Odom 超时根因已经修复。下一轮应优先在保持原门槛的前提下，对传感器适配器同步回调和 LIO 发布节奏做独立压力复测，再决定是否进入更长路线或硬件台架联调。
