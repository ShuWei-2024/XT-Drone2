# PX4 1.17 室外重复飞行、阻挡路径与失效记录

2026-10-14，ROS 2 Jazzy / Gazebo Harmonic / PX4 1.17，隔离 SITL。
这是同一个 Baylands 公园中不同路线和障碍形状的验证，尚非多地图通用性验证。
Baylands 是公共制作的模型，非现实扫描；坡地跟随、动态障碍、室外风场、真实光照和硬件未验证。

## 本轮结论

箱体正向、反向、斜向和圆柱阻挡路线已分别完成实际绕障；开阔矩形路线通过。
箱体最新复跑已完成，但仍有一条候选曲线被预览拒绝；扩大树旁路线的高度漂移和绕远问题仍需处理。
新增东侧 64.91 m 路线通过完整审计，西侧 53.75 m 任务完成但一处终点误差超标；
东侧同配置第二轮在首段前被规划器拒绝，因此重复稳定性尚未通过。
本轮全部尝试，包括失败和修复前的结果，见 [结构化记录](outdoor_repeat_ev_117.json)。
不同修复版本的成功不能合并成“同配置连续通过”；末点交接修复后的复跑结果单独记录。
累计 20 次飞行尝试，其中 7 次任务流程完成，其余为保护中止，全部降落上锁。
任务流程完成与全部精度审计通过分别记录：新增西侧任务完成，但未通过每航点 0.10 m 的精度门槛。
这些包含逐次修改配置的诊断实验，不能作为固定配置的成功率统计。

## 测试配置与测量口径

- 仅隔离 SITL：ROS domain 94、独立 Gazebo partition、`x500_0` namespace 和独立 PX4 rootfs。
  每轮结束核对原 PX4 参数/源码基线，以及运行中执行源码、二进制未改变。
- FAST-LIVO2、局部障碍地图、共享下降边界、EGO B-spline、受保护 Offboard 执行桥全部启用。
  规划速度 0.20 m/s、加速度 0.25 m/s²；巡航高度 0.90 m，初始航向校准爬升 1.60 m。
- EV 显式启用，`EKF2_EV_CTRL=3`，融合位置 XY/Z，不融合视觉航向和速度。
  保留 GPS、气压计与磁力计辅助；日志核对实际融合，不能只看参数。
- 时间配对不超过 20 ms，测量新鲜度不超过 250 ms；高度冻结参考漂移保护 0.20 m。
  XY 估计器差异保护 0.30 m；预先配置的室外跟踪保护为 0.60 m，非本轮为通过而放宽。
- 原点在地面冻结，飞行中不重拟合；Gazebo 真值用于审计和独立停测，不输入定位、规划或 EV 标定。
- 表中定位误差使用 `ground_frozen_errors.ego_execution`，去除一次地面常量参考差，
  不用空中重新拟合隐藏漂移。LIVO↔PX4 差值使用 EV 地面冻结变换，统计解锁期间。
- 实际障碍距离由 Gazebo 测量时间位姿和已知模型表面计算，是**机体中心到表面**的距离，
  不能当作桨尖净空；不认证未知树枝、缺失几何或动态障碍。

## 长路线失败与修复顺序

路线 `[[-7,0],[-14,0],[-21,0],[-22,10]]`，最后一段需要绕开预检查下降禁区。
这条路线没有证明空中直线会撞树，不能作为“树干挡路后成功避障”的证据。

| 运行后缀 `outdoor_tree_repeat_20261014_` | 到达航点 | 失败证据和处理 |
| --- | --- | --- |
| `01` | 1/4 | 约 169.068 s 局部地图仅剩 19 个体素，低于 20 个下限；LIO 仍有约 5,140 个 correspondence。5 m 局部地图裁掉了开阔区较远观测。 |
| `02` | 0/4 | 只扩大地图半径后达到 6,338 个体素，触发原 5,000 容量限制；半径和容量需要成对设置。 |
| `03` | 0/4 | 已完成的新地图尚未验收，健康检查先判旧发布物过期。修正先后顺序，仍检查新快照本身的新鲜度。 |
| `04` | 0/4 | PX4 测量出现约 52 ms 间隙，最新一对无法在 20 ms 内匹配。改从短历史选仍新鲜的实际测量对。 |
| `05` | 0/4 | 高度配对被无关的航向 reset 过滤；高度只按 Z reset 分代，航向继续独立保护。 |
| `06` | 0/4 | 地图扩大后膨胀计算在 SITL 负载下约 447 ms，导致过期；优化精确去重，不删障碍点。 |
| `07` | 2/4 | 高度监督已修复，但执行桥仍只选最新一对；将执行桥同步改为有年龄上限的真实历史配对。 |
| `08` | 3/4 | 地图和时间配对均正常，279.868 s 高度差超过冻结参考 0.20 m，保护降落。保留失败，不放宽高度门槛。 |

最终室外地图半径 12 m、最多 20,000 个体素；室内配置保持原值。
超过容量、输入过期或地图快照过期仍立即撤销 readiness。
膨胀使用有体积上限的布尔数组；巨大稀疏范围回退原去重算法，避免稠密分配失控。
同一批 12,593 个占据格输入，87,912 个输出格及排序完全一致；离线三次耗时由约 84 ms 降至约 24 ms。
这是桌面离线基准，不能当作 RK3588 实测结果。

`08` 的地面冻结真值审计：FAST-LIVO2 XY/Z p95 为 0.0316/0.1699 m，
PX4 XY/Z p95 为 0.0644/0.0522 m；EV 位置、高度 2,184/2,184 个日志样本实际融合且无拒绝。
因此“EV 已融合”仍不等于两套高度参考完全一致。
尚需分解 FAST-LIVO2 高度漂移、不同地面参考窗口和 PX4 EV 高度处理，当前不把任一项单独定为根因。

## 绕障和重复测试

| 运行 | 结果 | 实际航迹 / 最小表面距离 | FAST-LIVO2 真值 XY/Z p95 |
| --- | --- | --- | --- |
| `outdoor_box_ev_20261014_01` | 5/5，完整任务并降落 | 46.74 m / 1.305 m | 0.0411 / 0.0231 m |
| `outdoor_column_ev_20261014_01` | 4/4，完整任务并降落 | 23.56 m / 1.468 m | 0.0278 / 0.0138 m |
| `outdoor_box_ev_20261014_02` | 5/5，但任务收尾失败，保护降落 | 五点到达不能计整轮通过；最小距离 1.225 m | 详见结构化记录 |
| `outdoor_open_ev_20261014_01` | 修复交接后 4/4，完整任务并降落 | 22.57 m；开阔路线，不算阻挡绕障 | 0.0241 / 0.0239 m |
| `outdoor_box_ev_20261014_03` | 2/5，第三段重规划失效并保护降落 | 25.80 m / 1.270 m | 0.0671 / 0.0344 m |
| `outdoor_box_ev_20261014_05` | 1/5，第二段重规划失败并保护降落 | 未完成；无完整航迹审计 | 失败运行，详见 `planner.log` |
| `outdoor_box_ev_20261014_06` | 5/5，修复后完整任务并降落 | 48.57 m / 1.225 m | 0.0503 / 0.0325 m |
| `outdoor_column_ev_20261014_02` | 4/4，修复后完整任务并降落 | 23.86 m / 1.384 m | 0.0462 / 0.0255 m |

箱体路线 `[[7,0],[0,0],[7,3],[7,0],[0,0]]`，四段直线路径确实被已知模型阻挡，
审计比较实际横向绕行和最小距离，不把目标到达本身当作避障证据。
圆柱路线 `[[-7,0],[-7,3],[0,3],[0,0]]`，圆柱半径 0.70 m、高度 2.20 m。
它是树干/柱子的简化几何，用于测试圆形竖直障碍；不包含树冠、枝叶、细枝或真实树木扫描。

箱体 `02` 和 `01` 使用相同代码、配置；`02` 在 427.408 s 已到达五点，
随后先销毁执行接收器，再用单回调循环开始末点 hold。
约 158 ms 的 DDS 清理暂停后，配对消息已排队但监督器尚未处理，427.576 s 误触发高度配对保护。
该轮实际安全降落上锁，但 `flight_complete=false`，不计为完整成功。

修复：末点 hold 保留原有有界批量接收器，先处理待收回调再检查安全；
桥的输出定时器保持关闭，监督器独占固定末点 setpoint。
完成 hold 并采集结果后才销毁节点。检查测量年龄、配对、reset、地图、规划 heartbeat 和跟踪误差，
不重打时间戳、不插值补帧、不放宽门槛。
开阔路线已验证正常收尾；故障与排队回调顺序另有回归测试。

箱体 `03` 失败发生在 **285.164 s**，原因 `bridge fault: preview not tracking`。
当时地图有效、odom 测量年龄 44 ms；它尚未进入最终 hold，不是同一收尾故障复现。
规划日志多次拒绝曲线前 0.35～0.40 秒的 `XYZ≈[1.902,1.930,0.808]`，
最终出现 `First 3 control points in obstacles!` 并从 `REPLAN_TRAJ` 转为 `WAIT_TARGET`。
离线对最近记录地图和日志舍入坐标检查：该位置处于共享掩码禁格，却不在传感器膨胀占据格。
附近的实际 FAST-LIVO2 测量仍在掩码允许区内；已发布的 168 条完整曲线均通过边界、表面距离和动态约束审计。
因此目前证据指向拐角处的引导进度跟踪和预测起点，而不是实际碰撞；不能仅凭规划日志中的“obstacles”认定雷达看到新障碍。
复核源码后确认：引导进度窗口错过当前机体时，原实现会把局部目标留在旧进度，造成倒退前缀。
现已修复为在窗口错过时选择剩余引导线上最近且不倒退的点，保留原边界拒绝策略，需用新构建复测。
证据为该运行的 `replan_mask_failure.json` 与 `planner.log`。

箱体 `06` 和圆柱 `02` 使用引导进度修复及有界减速前缀复测，均完整到达所有航点。
箱体 `06` 仍有 1 条规划器发布的 B-spline 未通过离线共同边界曲线检查；预览节点记录了
`trajectory leaves shared planning/execution envelope` 并切换保持，执行桥随后接收有效曲线继续飞行。
因此这条曲线没有作为合格规划结果计入共同边界通过数；完整飞行审计确认实际执行航迹的动态约束、已知表面距离和保护条件通过。
这说明“规划器可能发布无效候选、执行预览拒绝它”的保护链有效，但规划器本身仍需消除这类候选。

## 复现

每轮先停止其他仿真，使用不存在的运行名称；以下命令会解锁**本机隔离 SITL**。

```bash
cd /home/cyf/dev/XT-Drone2-astra

# 开阔矩形路线
bash astra/run_117.bash obstacle outdoor_open_ev_new \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_park_open \
  --route-json astra/runtime/maps/baylands_park_open/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus

# 正向、反向、斜向箱体绕障
bash astra/run_117.bash obstacle outdoor_box_ev_new \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_reverse_validation \
  --route-json astra/runtime/maps/baylands_reverse_validation/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus

# 圆柱阻挡路线
bash astra/run_117.bash obstacle outdoor_column_ev_new \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_column_validation \
  --route-json astra/runtime/maps/baylands_column_validation/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus
```

飞行停止后逐项审计，全部结束再启动下一轮，避免离线分析争用仿真 CPU：

```bash
source astra/env_117.bash
python3 astra/tools/audit_scan_flight.py astra/runtime/运行名称
python3 astra/tools/audit_external_vision.py astra/runtime/运行名称
python3 astra/tools/audit_shared_envelope.py astra/runtime/运行名称
```

原始证据为各运行目录的 `motion_events.json`、`frame_samples.csv`、
`local_map_diagnostics.json`、`external_vision_sent.csv`、PX4 ULog、三项审计 JSON、
`receipt_summary.json`、`run_summary.json` 和 `runtime.json`。
本轮 144 项相关回归测试通过，包括末点排队回调、独占固定目标、失联/reset/跟踪故障保持生效、
真实历史时间配对、地图过期替换、精确膨胀、边界曲线和有界搜索。
Python 编译与 `git diff --check` 通过，`astra_fast_livo2` 和 `astra_px4_offboard` 包已重新构建；
最后末点交接修复位于直接运行的 Python 工具，无需重新安装 ROS 包。
20 次运行均记录原 PX4 基线保持、执行源码/二进制在运行中未改变；本轮进程已停止，未提交或推送。

RSS 增长与地图/视觉缓存随观测范围变化仍需更长测试；短路线图像缓存平台结论不能套用到宽范围建图。
没有运行同配置 EV 开/关对照，不能从这些不同路线直接量化 EV 的因果收益。

新增复杂环境和长距离分段审计见 [OUTDOOR_LONG_DRIFT_117.md](OUTDOOR_LONG_DRIFT_117.md)。
下一优先项是共享掩码预处理/搜索拒绝原因分解、树旁区域高度残差和终点精度，再进行当前配置的重复稳定性、独立室外地图、稀疏几何与光照条件测试。
暂不扩大到坡地、近障碍窄缝或实物飞行。
