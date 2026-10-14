# PX4 1.17：共享路线边界与 External Vision

2026-10-13，隔离 ROS 2 Jazzy / Gazebo Harmonic / PX4 1.17 SITL。
本轮完成规划器与执行保护使用同一套路线边界，并接入 FAST-LIVO2 的位置 External Vision。
四航点组合飞行通过，EV 节点失联保护通过；坡地跟随、整片公园任意飞行和实物未验证。
所有运行完成后已停止本轮进程，原 PX4 参数/源码基线及运行中的执行源码、二进制哈希保持一致。
未提交或推送本轮修改。

## 为什么之前规划成功但执行中止

旧规划器只判断实时障碍点云，执行保护还要求机体处于已检查的可下降区域。
某些空中直线不会碰撞，却穿过无法保证安全下降的区域，两者对同一路线的判定不同。
本轮保留下降保护策略，让规划器提前遵守它；这是项目当前的保守测试策略。

地面冻结 FAST-LIVO2 原点后，由同一份下降矩形证书生成 `/planning/execution_envelope`。
栅格分辨率 0.10 m、边界预留 0.40 m，并扣除覆盖整格所需的距离界。
该栅格是预先允许的安全区域，独立于传感器观测障碍；没有把模型几何伪装成雷达数据。
EGO 同时检查安全边界和实时障碍地图；缺失、非法或改变的边界使规划失效。
执行预览对每段三次 B-spline 转成 Bezier 曲线，递归检查整个控制点凸包，避免只采样端点漏检。
RViz 在起飞前使用同一个边界拒绝靠近禁区的目标。

树旁目标的允许绕行约 70 m，原局部 A* 搜索范围无法覆盖。
新增有时间预算的安全栅格全局搜索，生成引导折线，再由 EGO 连续规划局部轨迹。
全局引导只解决允许区域的连通关系；实时障碍、动态约束、定位和失联保护继续检查。
0.40 m 是设计预留，不能视为包含所有定位/跟踪误差的形式化安全证明。

## External Vision 接口与限制

显式 `--external-vision` 才启用，默认关闭。输入 `/fast_livo2/odom`，要求
`camera_init` / `base_link`；输出 `/x500_0/fmu/in/vehicle_visual_odometry`。

- ENU → NED、机体 FLU → FRD，四元数按 PX4 的 wxyz 顺序。
- 已知零航向 SITL 出生点，地面采集至少 4 秒、25 个时间配对样本；配对不超过 20 ms。
  冻结平移，不使用 Gazebo 真值标定，也不在飞行中重新拟合。
- `timestamp_sample` 保留原测量时间，`timestamp` 是发布时刻；不重复旧消息或预测位姿来补频率。
  输入通常约 10 Hz，启动阶段确实出现过间隙和门控撤销，不宣称全程无慢帧。
- `EKF2_EV_CTRL=3`：只融合 XY/Z；`EKF2_HGT_REF=3`。保留 GPS、气压计和磁力计辅助。
  不融合视觉速度和航向。odom 已是机体参考，不重复添加雷达安装偏移。
- 位置标准差暂用 0.10 m（方差 0.01 m²），是实验保守配置，尚非经过标定的估计协方差。
- 测量发布/监督 heartbeat 超时 250 ms。PX4 融合状态报文约 1 Hz，另用 1.5 s 看门狗，
  不把报文周期与测量新鲜度混为一谈。需要实际 EV 位置/高度融合标志才允许执行。
- 源位姿跳变锁存失效，需地面重启。EV 节点或数据失联时撤销执行，冻结目标并受控降落。

EV 参数仅写入本轮独立 rootfs，不修改常用 PX4 参数。完整临时参数见每轮 `runtime.json`。
接口依据：[PX4 外部位置估计说明](https://docs.px4.io/main/en/ros/external_position_estimation)。

## 实际结果

| 测试 | 结果与证据 |
| --- | --- |
| 无 EV 的共同边界树旁路线 | `baylands_shared_tree_117_01`：3/4 点到达，原局部搜索无法覆盖树区长绕行；高度差保护中止并降落。105 条曲线均在边界内，未计为成功路线。 |
| 全局引导离线回放 | `shared_envelope_tree_replay02`：约 70.2 m 引导，5 条曲线全部在边界内；这是规划回放，不是飞行。 |
| EV 静止 | `ev_static_117_01`：不解锁；PX4 日志确认位置/高度实际融合约 99.7%，无创新拒绝。 |
| EV 小范围运动 | `ev_small_motion_117_02`：起飞、前进、侧移、返回、降落通过；实际前进 ENU Y +0.463 m、侧移 ENU X +0.414 m；LIVO↔PX4 XY/Z 差值 p95 为 0.0656/0.0219 m。 |
| 共享边界 + EV 树旁组合 | `baylands_shared_tree_ev_117_01`：4/4 点完成，实际水平航迹 91.4 m，末点误差 0.0849 m，安全降落上锁。508 条完整曲线全部在共同边界内；已知模型表面最小机体中心距离 1.623 m。 |
| EV 节点失联 | `ev_loss_guard_117_01`：交接 2 秒后停止本轮 EV 节点，348.2 ms 后冻结保持，随后降落上锁；地图仍正常，验证了独立 EV 看门狗。 |

小范围首次 `ev_small_motion_117_01` 在解锁前失败：原融合状态看门狗 1 秒短于报文最大间隔约 1.024 秒。
调整为 1.5 秒后复测通过；测量新鲜度 250 ms 门限未放宽。

组合路线为 `[[-7,0],[-14,0],[-21,0],[-22,10]]`（起飞原点相对 XY）。
这是绕开树旁下降禁区的验证：直线空中路线并未证实会撞树，
因此 `known_surface_blocked_path_avoidance_verified=false`，不把大幅绕行冒充新增的阻挡路径避障证据。
此前箱子/圆柱阻挡路线的测试仍是另外的证据。

组合飞行的规划速度/加速度上界为 0.200 / 0.2482（限值 0.20 m/s、0.25 m/s²），
完整轨迹动态约束检查通过；合并 setpoint 最大发送间隙 136.6 ms，无空中 failsafe。
初始化和爬升期间存在重置：空中航向在 84.356 s 重置一次，reset-aware 门控处理后再交接执行；
不能称为“全程没有重置”。空中位置/高度 aid-source 日志 5784/5784 个样本实际融合，无创新拒绝，最大融合年龄 184 ms。

### 剩余精度差异

| 指标（地面冻结参考；不在空中拟合） | p95 | 最大值 |
| --- | --- | --- |
| LIVO↔PX4 XY 差值，解锁期间 | 0.1373 m | 0.1888 m |
| LIVO↔PX4 Z 差值，解锁期间 | 0.0866 m | 0.1378 m |
| LIVO↔PX4 航向差，解锁期间 | 1.556° | 1.813° |
| FAST-LIVO2↔Gazebo XY，执行期间 | 0.1305 m | 0.1811 m |
| PX4↔Gazebo XY，执行期间 | 0.1034 m | 0.1212 m |

测量配对 6118 个，覆盖率约 99.97%，p95 12 ms、最大 20 ms。
EV 确实融合，但 GPS 辅助、滤波和定位漂移仍使两套位姿不完全相同。
长路线尚未达到此前 XY p95 ≤0.10 m 的精确对齐门槛。
结论是允许区域中的这条低速路线通过，近障碍精度、重复稳定性、未知几何和硬件仍需单独验证。
Baylands 是公共制作的公园模型，不是现实扫描；坡地与地形跟随未加入。

## 复现命令

每次用未存在的运行名称，执行前停止其他仿真。所有 `small` / `obstacle` 命令会解锁隔离 SITL。

```bash
cd /home/cyf/dev/XT-Drone2-astra
# 当前已准备的组合路线，约 11 分钟仿真时间。
bash astra/run_117.bash obstacle shared_tree_ev_new \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_shared_tree \
  --route-json astra/runtime/maps/baylands_shared_tree/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus

# 小范围 EV 飞行
bash astra/run_117.bash small ev_small_new \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus

# EV 节点失联，预期保护退出，不是正常路线完成
bash astra/run_117.bash obstacle ev_loss_new \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_column_validation \
  --route-json astra/runtime/maps/baylands_column_validation/route.json \
  --calibrate-ground-height --external-vision --stop-ev-after-handoff-s 2 \
  --executor-audit --isolate-estimator-cpus

# 默认 10 m RViz 入口，中文窗口点击开始任务才起飞
bash astra/rviz_outdoor_117.bash outdoor_ev_new --external-vision
```

重新生成扩大路线证书（需已有 `baylands_park` 和地图工具环境；输出目录必须不存在）：

```bash
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands_expanded.py \
  --source astra/runtime/maps/baylands_park \
  --output astra/runtime/maps/baylands_shared_tree_new \
  --route-kind shared-tree --refine-descent-prisms
```

`shared-tree` 的单段超时 500 秒仅容纳长绕行；其他预设仍为 100 秒，数据新鲜度与动态约束不变。
准备不等于飞行认证，重新生成后应用新目录的路线执行并审计。

离线审计（停止仿真后）：

```bash
source astra/env_117.bash
python3 astra/tools/audit_scan_flight.py astra/runtime/baylands_shared_tree_ev_117_01
python3 astra/tools/audit_external_vision.py astra/runtime/baylands_shared_tree_ev_117_01
python3 astra/tools/audit_shared_envelope.py astra/runtime/baylands_shared_tree_ev_117_01
```

证据目录：`astra/runtime/<运行名称>/`，包括 `scan_flight_audit.json`、
`external_vision_audit.json`、`shared_envelope_audit.json`、`frame_samples.csv`、
`external_vision_sent.csv`、原始 B-spline、PX4 ULog 和 `run_summary.json`。
失联测试另有 `fault_regression.json`。大型运行数据不上传 Git。

## 构建与检查

本轮 `plan_env`、`path_searching`、`bspline_opt`、`ego_planner` 四包构建通过（约 2 分 24 秒，存在原有编译警告）；
共享边界 launch 参数所在的 `astra_fast_livo2` 包亦已构建。
本轮后续修复后的 125 项相关测试通过，包括连续曲线越界、掩码空洞、全局绕行/搜索超时、坐标与测量时间、
RViz 起飞前拒绝不满足共同边界的目标、下降证书与路线输入。
Python 编译、启动脚本语法和 `git diff --check` 通过。

## 2026-10-14 室外复测与修复

本节为当日阶段结果；后续箱体复跑又暴露拐角重规划问题，完整 13 次尝试、收尾交接修复和剩余限制见
[室外重复验证记录](OUTDOOR_REPEAT_EV_117.md)。不能把下表两次成功视为重复稳定性已经通过。

同一套 EV、规划和执行配置继续进行了两类真正被障碍物挡住的路线：

| 场景 | 结果 | 误差和距离 |
| --- | --- | --- |
| 橙色箱子，正向/反向/斜向 | 5/5 航点，约 46.7 m，安全降落 | 实际障碍最小机体中心距离 1.305 m；FAST-LIVO2 相对 Gazebo XY/Z p95 为 0.041/0.023 m；EV 融合后的 LIVO↔PX4 XY p95 为 0.054 m |
| 绿色圆柱 | 4/4 航点，约 23.6 m，安全降落 | 实际障碍最小机体中心距离 1.468 m；FAST-LIVO2 相对 Gazebo XY/Z p95 为 0.028/0.014 m；EV 融合后的 LIVO↔PX4 XY p95 为 0.056 m |

两轮的直线路径均被静态模型阻挡，审计确认发生实际绕行；速度上界 0.20 m/s、加速度上界约 0.245 m/s²，armed failsafe 为零。
这扩大了对不同障碍形状和方向的证据，但仍不代表任意树枝、窄通道或动态障碍都可通过。

复测也暴露并修复了三处室外运行问题：

1. 室外规划窗口为 10 m，而局部地图原来只保留 5 m；现为 12 m，并把有限容量从 5,000 提到 20,000 个体素。
   容量仍是硬上限，超限立即撤销 readiness。
2. 地图更新完成时，健康检查原先可能先判定旧地图过期，再丢弃已经完成的新地图；现先验收新鲜结果，再检查旧发布物。
3. 执行桥和高度监督都从短历史选择真实测量配对；配对误差仍不超过 20 ms、输入年龄仍不超过 250 ms，
   不插值、不重打时间戳，不用旧数据覆盖持续缺失。

同一路线的多次复测说明长距离仍受高度基准影响：一次新配置复测到 3/4 航点后，FAST-LIVO2 相对 Gazebo Z p95 约 0.170 m，
保护因 PX4/LIVO 高度漂移超过 0.20 m 而降落。它没有触发地图容量、地图过期或时间配对保护；这项失败保留为有效安全结果，不能把门限放宽来换取完成率。
此前 `baylands_shared_tree_ev_117_01` 的 4/4、91.4 m 记录仍有效，但当前长路线重复稳定性尚未通过。

同一批记录的离线体素膨胀对比保持 87,912 个输出体素完全一致，排序去重耗时约 84 ms 降到约 24 ms；这是离线基准，仍需在 RK3588 上重新测量。

```bash
source astra/env_117.bash
colcon --log-base log_117 build --build-base build_117 --install-base install_117 \
  --packages-select plan_env path_searching bspline_opt ego_planner \
  --parallel-workers 2 --event-handlers console_direct+
python3 -m pytest -q \
  astra/validation/test_planning_envelope.py \
  astra/validation/test_external_vision_geometry.py \
  astra/validation/test_envelope_path.py \
  astra/validation/test_rviz_goal_selection.py \
  astra/validation/test_outdoor_envelope.py \
  astra/validation/test_static_descent_columns.py \
  astra/validation/test_ground_height_alignment.py \
  astra/validation/test_measurement_pair_gate.py \
  astra/validation/test_sitl_route.py \
  astra/validation/test_scan_trajectory_audit.py
```
