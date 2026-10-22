# 空中估计器重置与完整 DDS 断链验收

日期：2026-10-20。隔离 PX4 1.17 / ROS 2 Jazzy / Gazebo Baylands，正常 FAST-LIVO2 图像融合、EV XY/Z、EGO-Planner 与 Offboard，约 0.20 m/s。故障在规划控制交接后、起点附近可下降区域注入；不代表任意位置恢复或实物验收。

## 注入与证据规则

- 实际重置：向 PX4 发送官方外部观测命令，并在机载 ULog 及直接遥测确认真实 reset 计数变化；不伪造 `/fmu/out` 消息或计数。XY 外部观测偏移约 0.05 m；航向约 2°。带高度的外部位置命令也可能同时重置 XY，不称作纯 Z 单因素测试。
- 完整 DDS 断链：只停止本轮拥有的 Micro XRCE DDS Agent，切断全部 PX4 DDS 输出以及 Offboard、命令和 EV 输入。Gazebo 传感器、PX4 EKF/控制器、机载日志及独立 MAVLink GCS 心跳继续运行；不等于所有物理通信或遥控链路同时失效。
- 断链后的 ROS 状态不能用于确认落地上锁。机载 `vehicle_status`、`vehicle_land_detected`、`failsafe_flags` 和收到的控制消息作为独立证据；“打印请求 Land”不等于 PX4 收到。
- 原 Odom 250 ms、测量配对 20 ms、高度差 0.20 m 等门槛保持；原 PX4 参数与二进制不修改。每轮独立保存基线校验及执行源码快照，故障任务必须中止，不能报告正常路线完成。

## 结果

**四项有效故障注入均完成预期保护并由机载日志确认落地上锁。** 它们是故障任务中止后的恢复通过，不是正常任务完成。

| 类型 | 有效运行 | 执行端检测 | 机载落地/上锁时间 | 结果 |
| --- | --- | --- | --- | --- |
| XY 重置 | `acceptance_reset_xy_20261020_01` | 36.65 ms | 3.784/5.796 s | 通过 |
| 含 Z 的 XY/Z 重置 | `acceptance_reset_z_20261020_03` | 21.38 ms | 3.968/5.980 s | 通过 |
| 航向重置 | `acceptance_reset_yaw_20261020_01` | 26.53 ms | 4.000/6.004 s | 通过 |
| 完整 DDS 断链 | `acceptance_dds_loss_20261020_02` | 211.06 ms | 4.140/6.152 s | 通过 |

检测/锁存使用实际单调墙钟，机载时间使用 PX4 消息时间。断链信号的仿真参考是之前最近一条完整在线记录，墙钟相隔 4.81 ms；机载相对时间因此是近似值，不能把它当作精确测得的 DDS 传输延迟。

XY、含高度、航向试验的机载计数分别为 XY `4→5`、Z `3→4`（同时 XY `4→5`）、heading `2→3`。命令均有 accepted 回执，机载 ULog 独立记录计数变化。四轮故障检测后正常 bridge 采样均为 0，任务完成标志均为 false。

四轮原 PX4 基线和飞行期间执行源码均保持；注入工具在调试轮之间有修改，因此不是四次同源码重复。相对首轮的变化仅为 `run_sitl_117.py`、`sitl_estimator_reset.py`、`sitl_extended_faults.py`，逐轮差异写入汇总 JSON；定位、规划和执行保护算法未在本批修改。

记录中的最小已知表面机体中心净空分别为 2.620/2.880/2.867/2.681 m；这是已采样记录下界，不替代连续旋翼净空认证。信号发出后，Gazebo 水平移动最大约 0.123/0.106/0.114/0.304 m，说明检测和停止不是瞬时制动。

## 完整 DDS 断链的机载证据

断链后独立 ROS 记录没有继续收到 PX4 位置/状态；PX4 机载记录也不再收到新的 Offboard 心跳，未收到断链后地面尝试发送的 Land 命令。与此同时 Gazebo 真值与 PX4 本地估计继续更新，最终由机载自动模式完成降落。

- `offboard_control_signal_lost` 首次置位约 1.036 s，`vehicle_status.nav_state` 进入 AUTO_LAND 约 1.048 s；约 4.140 s 检测落地、6.152 s 上锁。
- 当时 `manual_control_signal_lost=true`、`global_position_invalid=true`，本地位置/高度有效，GCS 连接未丢失。源码中默认位置模式会在无手动控制时进入 RC 失联动作与后续模式可用性回退；本轮结果是 Land，不能外推为任意 GPS/遥控状态都一定 Land。
- 实际日志参数为 `COM_OF_LOSS_T=1.0`、`COM_OBL_RC_ACT=0`、`NAV_RCL_ACT=2`、`COM_FAIL_ACT_T=5.0`、`COM_RC_IN_MODE=1`，本批未为测试临时改这些参数。
- ROS 任务结果 `disarmed=false` 表示链路已断、无法确认；没有改成 true。离线审计根据 ULog 确认上锁，分别保存两种结论。

完整证据见[汇总 JSON](estimator_link_faults_117.json)。

## 已保留的测试入口失败

`acceptance_dds_loss_20261020_01` 在发出断链信号之前，误读了结束时才导出的 `frame_samples.csv`，运行器提前清理。该轮没有有效注入，不计为保护通过。已改用逐行落盘的在线 JSONL 尾部，拒绝过期锚点；注入失败只记录失败并让原监督器继续任务，相关回归覆盖缺失文件与不完整尾行。

`acceptance_reset_z_20261020_01` 在注入器冷启动期间出现 EV 发布间隔 >250 ms，原保护先行中止任务并正常上锁，未发送重置命令。核心记录相邻 Odom 发布有约 433 ms 墙钟缺口，而前后 LIO 处理约 20～28 ms；这不是该帧算法计算耗时。注入器启动与缺口时间重叠，不能据此证明所有此类缺口的根因。改为地面预建 DDS 参与者，空中只创建触发标志；后续不复现不等于消除了所有超时。

`acceptance_reset_z_20261020_02` 完整飞完短路线并上锁，但注入器把约 0.5 s 发布一次的事件型 status 错套 0.3 s 门槛，拒绝发命令，因此不计为重置通过。现与原监督器保持一致：位置 250 ms、status 1 s；另有触发前快照和回归检查。最终有效高度/航向测试使用这个入口。

## 修改与软件验证

仅扩展隔离测试运行器和两项测试工具，没有调整定位参数、飞行保护门槛或 PX4 二进制。注入器在地面就绪后才允许启动飞行监督器，注入异常不直接拆除在飞仿真；完整断链任务的高频日志只在离线阶段重建。

最终 Python 回归 **382 passed**，保留三条既有 protobuf/时间 API 弃用提示；语法检查与 `git diff --check` 通过。本批无 C++ 修改，未重新构建算法。

## 复现

以下路径为本机已准备场景和短路线，使用新运行名：

```bash
bash astra/run_117.bash obstacle reset_NEW \
  --world-preset baylands \
  --scan-dir astra/runtime/maps/baylands_coloured_short_retest \
  --route-json astra/runtime/final_px4_position_loss_20261019_01/route.json \
  --calibrate-ground-height --external-vision --executor-audit \
  --isolate-estimator-cpus --execution-executor events \
  --reset-estimator-after-handoff-s 2 --reset-estimator-axis xy
```

其他重置使用 `--reset-estimator-axis z` 或 `yaw`；完整 DDS 断链用 `--stop-dds-agent-after-handoff-s 2` 替代两个 reset 参数。故障类型互斥。重置注入器只能在本运行器的隔离子进程环境中启动。

每轮机载日志正常关闭后，运行器自动产生 `extended_fault_audit.json`。原始日志位于 `astra/runtime/<运行名>/`；仓库汇总随后写入本报告。

## 适用边界与后续

后续[多位置故障恢复](MULTILOCATION_FAULTS_117.md)已补齐远端完整 DDS 断链、障碍旁地图停止、不可下降通行区地图停止各一次。远端全局位置有效时 PX4 选择 RTL；因此本报告起点附近的 Land 不能直接外推。任意位置与更广的重置／失联组合仍未验证。

本项补齐起点附近可下降区域的真实 XY、含高度、航向重置及完整 DDS Agent 失联保护证据。大幅、多次重置、全区域恢复、单向链路故障、独立遥控/GCS 同时失联、无 GNSS 辅助以及真实链路仍未覆盖；本次默认失联动作下的结果不能替代这些场景。

正常配置三轮长路线与五分钟保持证据见[重复验收](NORMAL_LONG_ACCEPTANCE_117.md)。仍需继续处理扩图/长时内存、多位置恢复与历史定位漂移；不因本项通过宣称所有 Odom 超时已根治或可直接上桨飞行。
