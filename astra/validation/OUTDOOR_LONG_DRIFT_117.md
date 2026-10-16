# 复杂室外路线与长距离漂移审计

2026-10-14，隔离 PX4 1.17 SITL，Baylands 公园。新增四次运行，全部受控降落并上锁，
原 PX4 参数/源码基线保持，运行中执行源码和二进制不变。没有改定位、规划或飞控算法来增加完成率。
本轮测试新增代码仅为离线漂移审计及其四项回归测试；后续随当前集成代码和文档整理发布。

## 结果

以下误差使用起飞前冻结的一次平移和固定 ENU/NED 轴变换，统计 EGO 执行阶段。
没有空中重新标定原点；实际路线长度使用测量时间的 Gazebo 机体中心轨迹。

| 运行 | 任务流程 | 实际水平航迹 | LIVO 真值 XY/Z p95 | 完整精度与安全审计 |
| --- | --- | --- | --- | --- |
| `outdoor_tree_refined_ev_20261014_09` | 3/8，第四段超时后降落 | 39.33 m | 0.0687 / 0.0955 m | 未通过 |
| `outdoor_east_ev_20261014_01` | 8/8，完成后降落 | 64.91 m | 0.0433 / 0.0216 m | 通过 |
| `outdoor_west_ev_20261014_01` | 8/8，完成后降落 | 53.75 m | 0.0662 / 0.0142 m | 未通过：第四航点终点误差 0.1224 m |
| `outdoor_east_ev_20261014_02` | 0/8，没有轨迹，保持后降落 | 无 EGO 执行航迹 | 不作为运动精度证据 | 未通过 |

西侧虽然完成所有目标，但 PX4 相对真值的地面冻结 XY p95 为 0.1104 m，
有一处航点的真值终点误差超过原 0.10 m 门槛。不能用“退出码为零”代替全部指标通过。
东侧最后航点真值误差 0.0305 m；西侧最后航点 0.0372 m。

东侧首段直线被箱体阻挡，实际最大横向偏离 3.0385 m，目标到达；
实际最小已知表面距离 1.4852 m，完整绕障审计通过。西侧是开阔对照，不能作为树枝绕障证据。
距离是机体中心到已知模型表面，不能当作桨尖净空。

前三轮所有已执行曲线通过共同安全边界，连续表面距离和规划动态约束检查通过；
第四轮没有曲线，按未验证处理。四轮 armed failsafe 均为零。
规划曲线速度上限 0.20 m/s、加速度上限 0.25 m/s²不变；东/西实际三维峰值速度
分别约 0.264/0.261 m/s，包括跟踪和高度变化，不能把规划曲线限速说成实际飞行的严格上限。

## 长距离高度分解

新工具 `astra/tools/audit_long_route_drift.py` 复用 `scan_flight_audit.json` 的地面冻结偏移，
按传感器测量时间配对 Gazebo、FAST-LIVO2 和 PX4，最大配对间隔 20 ms。
每个航段输出 signed Z 的首秒/末秒中位数、绝对误差 p95、实际路径、配对率、监督高度差。
起飞、下降到巡航高度、末点 hold、降落分别统计，未交接执行的测试标记为 `planner_wait`。
两个不同时间窗口冻结的参考分别保留：真值误差、监督器高度偏移、EV 发布变换不能混用。

| 运行与区域 | LIVO signed Z（首秒 → 末秒） | 解释 |
| --- | --- | --- |
| 旧 `outdoor_tree_repeat_20261014_08` 第一段 | −0.0101 → −0.0591 m | 高度偏差随该段增加 |
| 同轮第二段 | −0.0587 → −0.1028 m | 继续增加 |
| 同轮第三段 | −0.1022 → −0.1693 m | 累积至约 17 cm |
| 同轮第四段 | −0.1685 → −0.1682 m | 在附近保持较大偏差，最后触发监督高度保护 |
| 新树旁 `09` 第三段 | −0.0720 → −0.0994 m | 同区域出现较小但相似累积 |
| 新树旁 `09` 第四段返回方向 | −0.0973 → −0.0283 m | 回到已经过区域后误差减小 |
| 东侧 `01` 完整执行 | −0.0107 → −0.0125 m | 没有持续单向累积 |
| 西侧 `01` 完整执行 | −0.0155 → −0.0159 m | 没有持续单向累积 |

这些数据支持“高度误差存在区域/观测相关性”，尚不能确定是 VIO、平面拟合、IMU 或地图处理中的哪一项。
EV 融合正常也不能证明定位真值误差已经消除。东/西 EV 地面冻结 LIVO↔PX4 Z p95
分别为 0.0408/0.0348 m；XY p95 分别 0.0997/0.1113 m。
EV 测量配对 p95 12 ms、最大 20 ms；Gazebo 真值配对 p95 8 ms。
PX4 日志确认 XY/Z 融合，未融合视觉航向/速度。
各轮仍有一次解锁后的航向 reset，发生在正式 EGO 执行前；详细事件保留于 EV 审计。
全程 odom 接收峰值间隔和年龄存在偶发尖峰，不能由 p95 低就认证每一帧都实时。

## 树旁超时与东侧重复失败

树旁 `09` 的 `planner.log` 记录第四段 `Shared-envelope global guide: 70.423 m, 29 corners`。
两点直线间隔约 10 m，但整个下降安全区域约束迫使当前引导大幅绕行。
按名义 0.20 m/s 仅走完引导至少要 352.1 s，而该场景 `planner_leg_timeout_s=100`。
实际在 326.916 s（仿真时间）中止。不能把这次超时归因于计算性能或轴方向反转，
也没有证明不存在更短的安全路线。没有延长期限、放宽边界或取消保护来让本轮通过。

东侧 `02` 使用与 `01` 相同执行算法和场景配置，首个目标被拒绝：
`No bounded route in shared execution envelope; goal rejected`，直到 206.112 s 超时降落。
没有 B-spline，没有 planner handoff，不计一次成功重复。
`GridMap::executionEnvelopeGuide()` 将掩码逐格扩展检查和拓扑搜索共用 20 ms 墙钟预算；
日志未分别记录超预算、不可达和端点被额外 padding 排除，当前无法从这条错误确定唯一根因。
离线用冻结安全掩码、相同两格 padding 和最近记录起点找到约 13.0 m 的四连通通路。
该起点与请求时刻相差 32 ms，并非精确 planner 回调状态；因此只作为拓扑线索，
不代替 20 ms 测量配对认证，也不复现运行时调度。
下一步应先补齐拒绝原因和阶段耗时，再优化等价掩码处理，保持现有安全阈值。

## 内存与验证范围

东侧 FAST-LIVO2 RSS 约 203 → 469 MiB，西侧约 201 → 464 MiB，包含初始化和图像缓存预热。
单次约 6～8 分钟、几十米路线不能证明宽范围长期内存平台。
这些仍是同一公共制作的公园模型中的不同区域，不是独立地图、真实扫描、风场、细枝、动态障碍或坡地验证。
本轮四次尝试中两次完成任务，但仅东侧第一轮通过完整航点精度审计；不能报告“重复稳定通过”。

## 命令与证据

```bash
cd /home/cyf/dev/XT-Drone2-astra
# 本轮用下列三种 SCENE；每次须使用新 RUN，按顺序运行。
# baylands_expanded_tree_refined / baylands_expanded_east / baylands_expanded_west
bash astra/run_117.bash obstacle RUN \
  --world-preset baylands --scan-dir astra/runtime/maps/SCENE \
  --route-json astra/runtime/maps/SCENE/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus

# 结束全部仿真后审计。
source astra/env_117.bash
python3 astra/tools/audit_scan_flight.py astra/runtime/RUN
python3 astra/tools/audit_external_vision.py astra/runtime/RUN
python3 astra/tools/audit_shared_envelope.py astra/runtime/RUN
python3 astra/tools/audit_long_route_drift.py astra/runtime/RUN
python3 -m pytest -q astra/validation/test_long_route_drift.py
```

每次运行目录保存 `long_route_drift_audit.json`、`long_route_drift.png`、原始位姿 CSV、
测量/处理时间记录、PX4 ULog、规划日志、三份常规审计与基线校验。
汇总见 [outdoor_long_drift_117.json](outdoor_long_drift_117.json)。
四项新测试覆盖地面冻结误差不被重新拟合消除、NED 轴/Z/平移、20 ms 严格配对边界、
未执行失败不误标为运动；Python 编译及 `git diff --check` 通过。
