# 不同方向和障碍形状验证（2026-10-13）

继续使用同一套 FAST-LIVO2、局部地图、EGO 和 PX4 1.17 SITL 链路，
没有给规划器注入仿真模型地图或绕行中间点。原安全条件与速度限制保持不变。
两轮完整飞行都完成了离线距离、动态约束、实际位移、落地和基线保存检查。

| 场景 | 航点完成 | 实际水平路径 | 最小已知障碍距离（机体中心） | 峰值规划速度 / 加速度 |
|---|---:|---:|---:|---:|
| 箱子正向、反向、斜向、再次反向 | 5/5 | 41.08 m | 1.179 m | 0.200 m/s / 0.216 m/s² |
| 新增圆柱障碍 | 4/4 | 22.07 m | 1.279 m | 0.200 m/s / 0.203 m/s² |

箱子路线是 `[7,0] → [0,0] → [7,3] → [7,0] → [0,0]`，
其中四段直线被障碍挡住；测量结果确认四次实际绕行。
圆柱场景在复制的公园中增加半径 0.70 m、高 2.20 m 的障碍，
中心 `[-3.5,0,1.1]`；保留原公园与箱子，重新计算下降净空区域。
圆柱路线是 `[-7,0] → [-7,3] → [0,3] → [0,0]`，第一段必须绕行。
圆柱只是曲面测试道具，不用它替代树木细枝、遮挡或动态障碍的验证。

实际路径对比图：`astra/runtime/baylands_generalization_117/actual_detours.png`。
结构化结果：[outdoor_generalization_117.json](outdoor_generalization_117.json)。

## 本次暴露并修复的问题

首次反向试验 `baylands_reverse_117_01` 在完成正向目标后规划失败。
日志显示多个优化尝试重复相同直线初值；80 ms 预算耗尽，三个失败回调后撤销目标。
飞机保持已完成的终点，100 s 单段期限到达后受控降落，未触发 armed failsafe。
这不是“物理上没有路”，而是规划器没有正确尝试已有备选。

失败重试原先依赖“连续状态转换次数”，但失败回调没有再次执行状态转换，
该计数保持为一，因而始终不选择网格/随机备选。
现在使用实际失败次数和当前尝试索引：首个普通种子失败后切换备选，
预算耗尽立即退出循环；下一次回调重新获得原有 80 ms 预算。
没有增加超时、取消碰撞检查或提供模型真值路径。

另一个状态问题是正常完成会撤销 trigger。新仿真目标通过原 readiness、
odom 和坐标检查后重新触发规划；真实实验模式仍保留显式触发要求。

修改后先回放第一次失败时的记录地图和反向目标，再实际飞行。
回放只验证规划输出，不认证飞行；它的稀疏地图与实时地图存在采样差异。
实际复测 `baylands_reverse_117_02` 完成五个目标，日志确认网格备选被使用。
`baylands_column_117_01` 使用相同修复和参数完成四个目标。

两轮所有输出曲线均通过连续距离下界和动态审计。
执行期间规划心跳接收最大间隔分别为 63.4 / 60.9 ms；
setpoint 最大发送间隔为 90.1 / 72.8 ms；armed failsafe 都为零。
原 PX4 基线、飞行期间执行源码均保持不变，测试结束已解除解锁并停止仿真。
C++ 核心重新构建通过，全部 145 项测试通过（3 warnings）。

## 复现

原场景和路线文件保留在忽略的 `astra/runtime/maps`，本次实际命令为：

```bash
cd ~/dev/XT-Drone2-astra
bash astra/run_117.bash obstacle baylands_reverse_117_02 \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_reverse_validation \
  --route-json astra/runtime/maps/baylands_reverse_validation/route.json \
  --calibrate-ground-height --executor-audit --isolate-estimator-cpus
bash astra/run_117.bash obstacle baylands_column_117_01 \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_column_validation \
  --route-json astra/runtime/maps/baylands_column_validation/route.json \
  --calibrate-ground-height --executor-audit --isolate-estimator-cpus
```

运行目录拒绝覆盖，重测时必须使用新名称；两轮顺序执行。
仿真和 GUI 结束后分别执行 `python3 astra/tools/audit_scan_flight.py <运行目录>`。
圆柱场景可从已准备的原场景重新生成到新目录：

```bash
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands_column_validation.py \
  --source astra/runtime/maps/baylands_park \
  --output astra/runtime/maps/baylands_column_check
source astra/env_117.bash
ROS_DOMAIN_ID=95 python3 astra/tools/replay_planner_inputs.py \
  astra/runtime/baylands_reverse_117_01 astra/runtime/reverse_replay_check \
  --goal-index 1 --seconds 6
python3 -m pytest -q astra/validation
```

## 验证范围

修复的是通用规划重试与任务状态逻辑，同一套代码可以处理不同障碍。
“能绕”仍要求目标和路径有足够净空、传感器能及时观测、地图和位姿可信、
规划计算在预算内完成。封闭墙面或净空不足的通道可能没有可行路径。
不能把算法找不到路等同于已经证明没有路。

当前证据覆盖上述箱子和圆柱的静态场景，以及原先通过的路线。
不认证任意障碍、全部树枝、窄通道、动态物体、整片公园、真实飞行或长期可靠性；
“确实无可行路径”场景本轮尚未专门验证。
