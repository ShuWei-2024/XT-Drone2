# Baylands 固定配置收尾复测

日期：2026-10-21。范围为隔离 PX4 1.17 / ROS 2 Jazzy / Gazebo Baylands，使用正常 FAST-LIVO2 图像融合、EGO-Planner、Offboard 执行和 PX4 External Vision 的 XY/Z 融合。本报告只描述仿真指定路线，不是实物放飞许可，也不是任意地图认证。

## 配置与命令

短路线与第 6 轮长时运动使用同一份 129 项执行源码/二进制 manifest、同一估计器 YAML、PX4 覆盖参数、CPU 隔离和事件执行器。第 4 轮没有解锁，单独按启动门控记录。原保护门槛没有放宽：Odom 测量/接收年龄 250 ms、测量配对 20 ms、高度差 0.20 m、已知表面机体中心净空 0.55 m。

```bash
bash astra/run_117.bash obstacle acceptance_baylands_20261021_05 \
  --world-preset baylands \
  --scan-dir astra/runtime/maps/baylands_gc_receiver_20261018 \
  --route-json astra/runtime/maps/baylands_gc_receiver_20261018/route.json \
  --calibrate-ground-height --external-vision --executor-audit \
  --isolate-estimator-cpus --execution-executor events \
  --end-action return_home --endpoint-hold-seconds 300
```

每轮运行目录中的 `execution_sources/manifest.json` 均为 129 项；第 5 轮与第 1～3 轮逐项哈希相同。运行目录、PX4 参数备份、ULog 和完整 CSV 保存在本机 `astra/runtime/`，不随仓库分发。

## 运行结果

| 运行 | 结果 | 实际水平路长 | 航点 | 末点误差 | 最小已知表面净空 |
| --- | --- | ---: | ---: | ---: | ---: |
| `..._01` | 完成、返航、300 s 保持、降落上锁 | 71.321 m | 3/3 | 返航后 PX4 端点误差 8.30 cm | 1.212 m |
| `..._02` | 完成、返航、300 s 保持、降落上锁 | 71.433 m | 3/3 | 返航后 PX4 端点误差 1.69 cm | 1.215 m |
| `..._03` | 高度保护中止后受控降落上锁 | 40.595 m（中止前） | 1/3 | — | 1.265 m |
| `..._04` | 未解锁；EV/全局原点初始化位置变化超过 0.03 m | — | — | — | — |
| `..._05` | 完成、返航、300 s 保持、降落上锁 | 71.229 m | 3/3 | 返航后 PX4 端点误差 1.26 cm | 1.261 m |
| `..._06` | 长时运动因 Odom 测量年龄保护中止，受控降落上锁 | 56.416 m（中止前） | 2/3 | — | 1.263 m |

第 5 轮的航点实际到达时间为仿真 `183.188 / 394.612 / 558.752 s`，返航保持结束于 `856.388 s`，`862.112 s` 已确认上锁。测量时间真值路径为 `71.238 m`；规划曲线 422 条，规划峰值速度 `0.200 m/s`、加速度 `0.248 m/s²`，全程共享空域无越界，实际峰值三维速度 `0.269 m/s`，无 armed failsafe。第一个阻挡直线路径产生了实测绕行，三个目标均到达。

第 5 轮按起飞前冻结真值参考统计，飞行期间 FAST-LIVO2 XY/Z 误差 p95 为 `0.088/0.043 m`，最大 XY 为 `0.129 m`；PX4 XY/Z p95 为 `0.050/0.041 m`。三个目标的实际端点 XY 误差分别为 `3.02/3.08/4.98 cm`。这是本轮特定地图和低速路线的实测精度，不是全图精度承诺。

因此短路线固定配置的正常飞行尝试为 **4 轮，其中 3 轮完成、1 轮触发既有高度保护**；第 4 轮是地面启动门控失败，不能算作飞行失败。第 6 轮是独立的 1500 s 长时运动试验，进入飞行后在 `487.580 s` 因 Odom 测量年龄 `268 ms` 触发既有保护，不能计入正常路线完成次数。任务流程已有重复成功，但长时延迟尾部和高度问题仍未解决。

## 时效、融合与高度

第 5 轮健康审计通过：

- FAST-LIVO2 测量年龄 p95/max：`56/128 ms`；接收记录中的 Odom 时钟年龄 p95/max：`64/188 ms`；
- Gazebo 真值与 PX4 测量时间配对 p95/max：`8/12 ms`，全部配对在 20 ms 内；
- Odom、相机、LiDAR、IMU、PX4 本地位置和地图 readiness 在收尾阶段均有数据；
- PX4 armed 窗口中的 External Vision 位置和高度 aid-source 融合比例均为 `100%`，无视觉航向/速度融合；
- 固定地面变换下，LIVO/PX4 的 XY 差 p95 为 `0.071 m`。绝对 Z 差 p95 为 `0.284 m`，但固定 PX4 原点后相对 Z p95 为 `0.046 m`；这说明高度基准仍应单独验收，不能把本轮完成路线写成高度精密对齐通过；
- 高度动态离线审计中 LIVO/PX4 对真值的绝对误差 p95 为 `0.039/0.052 m`，实际保持高度范围为 `0.237 m`。这不能证明历史高度漂移根因已经修复。

## 内存与审计开销

第 5 轮约 14 分钟的有限窗口中，LIVO RSS 从约 `214 MiB` 到 `496 MiB`，增量约 `281.6 MiB`；其中保留图像最高约 `513` 张、`150.3 MiB`，已有图像/观测淘汰计数。LIVO 最后 120 s 变化约 `−0.086 MiB`，显示本窗口后段接近平台，但离无限运行或更大地图稳定仍有距离。此前把 motion 执行进程全记录窗口的 RSS 增量 `179.4 MiB`、末 120 s 增量 `52.0 MiB` 当作空中增长，统计窗口实际混入落地后的审计导出峰值。按 ULog 空中窗口核对，motion RSS 约从 `180.2 MiB` 增至 `185.6 MiB`，末 120 s 增约 `0.125 MiB`；落地后导出峰值约 `271.3 MiB`。这只说明第 5 轮的空中窗口接近平台，不能证明长时间或全栈内存稳定。

第 6 轮长时运动中，LIVO 从解锁后约 `203.6 MiB` 增到中止前约 `468.6 MiB`，最后 120 s 增约 `1.9 MiB`；控制/审计进程空中窗口增约 `43.1 MiB`，落地后的审计导出阶段短暂峰值约 `236.2 MiB`，不能当作空中增长。第 6 轮的详细延迟、航迹和内存拆分见[Baylands 长时运动中止审计](BAYLANDS_LONG_MOTION_20261021.md)。这些结果仍不能把全栈内存称为长期稳定。

## 结论与下一步

当前可以确认：低速 Baylands 指定路线能够起飞、连续到达三个航点、绕过已知障碍、返航、保持并安全降落；时间配对、EV XY/Z 融合和路线安全检查在第 5 轮均有证据。

当前不能确认：

1. 历史 LIVO/PX4 高度漂移根因已修复；
2. 详细审计开启时全栈内存可长期稳定；
3. 任意地图、未知几何、坡地、风或更高速度下仍保持同样结果；
4. RK3588 和真实机体的时钟、外参、算力、遥控接管及失联行为。

收尾顺序保持为：先针对第 6 轮暴露的传感器同步回调缺口和 LIO 发布节奏做独立压力复测，再做高度漂移和跨区域重复；通过后进行不装桨硬件联调，首次真实飞行仍需独立门控。EGO-Planner 执行桥保持仿真专用配置，不直接外推到实机。

## 证据文件

- `astra/runtime/acceptance_baylands_20261021_05/motion_events.json`
- `astra/runtime/acceptance_baylands_20261021_05/scan_flight_audit.json`
- `astra/runtime/acceptance_baylands_20261021_05/external_vision_audit.json`
- `astra/runtime/acceptance_baylands_20261021_05/height_dynamics_audit.json`
- `astra/runtime/acceptance_baylands_20261021_05/runtime_growth_audit.json`
- `astra/runtime/acceptance_baylands_20261021_05/log/2026-10-21/12_10_26.ulg`
- `astra/runtime/acceptance_baylands_20261021_06/odom_delivery_audit.json`
- `astra/runtime/acceptance_baylands_20261021_06/executor_timing_audit.json`
- `astra/runtime/acceptance_baylands_20261021_06/scan_flight_audit.json`
- `astra/runtime/acceptance_baylands_20261021_06/runtime_growth_audit.json`
- `astra/runtime/acceptance_baylands_20261021_06/log/2026-10-21/13_25_48.ulg`
