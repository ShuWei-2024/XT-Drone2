# 扩大室内路线与横向专项测试（2026-10-11）

本轮新增五次独立、正常传感器噪声的隔离 PX4 1.17 SITL 飞行。扩大四段
绕障路线 **2/2 严格通过**，横向专项 **1/1 通过**；更长的七段路线
**0/2 通过**。大范围运动能力已有证据，长路线重复稳定性仍未通过。
全部正常降落、解除解锁、退出仿真，原 PX4 1.15 基线保持，执行期间
源码/二进制哈希不变。没有真机、External Vision 或 Git 提交/推送。

完整数值：[large_indoor_117.json](large_indoor_117.json)。原始 ULog、测量
时间位姿、事件、资源与配置快照位于 `astra/runtime/<运行名称>/`。
独立逐轮结论位于该目录的 `indoor_flight_audit.json`。

## 实际结果

| 运行名称 | 完成目标 | 严格结果 | 真值水平航程 m | 全程已知障碍最小距离 m | 发布轨迹连续距离下界 m |
| --- | ---: | --- | ---: | ---: | ---: |
| indoor_large_events_117_01 | 4/4 | 通过，阻挡直线路径自主绕行 | 33.155 | 0.952 | 0.870 |
| indoor_large_events_117_02 | 4/4 | 通过，同路线独立重启重复 | 32.981 | 0.937 | 0.891 |
| indoor_large_lateral_events_117_01 | 2/7 | 第三段高度差保护停测 | 20.801 | 0.891 | 0.834 |
| indoor_large_lateral_events_117_02 | 3/7 | 第四段 odom 新鲜度保护停测 | 33.310 | 0.949 | 0.905 |
| indoor_lateral_only_events_117_01 | 3/3 | 通过，单独横向穿越及返回 | 16.510 | 1.228 | 1.240 |

四段路线偏移 `[[6,2.8],[0,0],[6,-2.8],[0,0]]`，仅给目的地，规划器
从实时局部地图绕过阻挡直线的箱子。两轮真值 XY 覆盖约 6.1×5.6 m，
已到达端点真值 XY 最大误差分别 0.0662 / 0.0577 m。规划动态约束、连续
规划距离、实飞距离和交接保持检查通过。这是合成室内场景，不能替代
真实扫描办公室或无限运行认证。

## 横向专项

专项偏移 `[[-0.5,4],[-0.5,-4],[0,0]]`。三段实际横向位移分别
`+4.025 / -8.005 / +3.994 m`，真值 XY 覆盖 0.566×8.036 m，最大端点
XY 误差 0.0381 m。该路线直线本身畅通，不计为新增阻挡绕障证据。

按测量时间配对，LIVO/Gazebo 与 PX4/Gazebo 时间差 p95 均 8 ms。
地面冻结原点后的执行期误差 p95：

| 估计器 | XY m | Z m | yaw 度 |
| --- | ---: | ---: | ---: |
| FAST-LIVO2 | 0.00483 | 0.06281 | 0.0220 |
| PX4 EKF | 0.05268 | 0.05725 | 1.2246 |

不是重新拟合全程原点消除漂移，也没有使用 Gazebo 真值生成 setpoint。
本轮不能代替完整坐标精度认证，独立审计仍保留 `frame_alignment_certified=false`。
执行期无 XY/Z/yaw reset、armed failsafe 或 EV 融合。

odom 测量年龄 p95 132 ms、最大 200 ms；合计 setpoint 最大发送间隔
78.1 ms。LIO 墙钟 p95 36.5 ms、最大 77.0 ms，CPU 最大 35.3 ms，调度
等待最大 42.4 ms。接收批处理诊断尾段线程 CPU/墙钟约 29.4%，不是整机
CPU。LIVO 最终 RSS 419.45 MiB，最后一分钟增长约 0.004 MiB；这只说明
本轮平台附近内存稳定。

## 长路线失败，分别保留

七段路线在四段后追加横向专项，两轮均未到新增横向目标。

第一轮约仿真 **279.956 s**，LIVO/PX4 相对地面冻结高度差超过 **0.20 m**。
当时 odom 年龄约 48 ms，地图和航向正常。最后两秒，相对各自地面冻结
真值原点，LIVO 有符号高度残差中位数 −0.0111 m；PX4 +0.1189 m，最大
+0.1762 m。证据指向 PX4 高度残差增大，尚不能确认气压计是唯一根因。
没有修改气压计噪声或放宽高度阈值。另有 height dynamics 报告使用空中
冻结参考，不能与本报告的地面冻结原点混用。

第二轮约仿真 **388.668 s**，odom 测量年龄 **256 ms**，超过原 **250 ms**。
附近慢 LIO 帧墙钟 **117.158 ms**、线程 CPU **33.106 ms**、调度等待
**72.667 ms**。独立订阅者收到较新 odom，监督器最后保存的位姿却仍较旧。
较新消息抵达时间接近有界回调批处理结束，存在主机调度竞争与批处理/
新鲜度决策边界共同影响的线索，尚未确认单一根因，也未撤销有效保护。

旧 `test_aborted` 在 ROS teardown 后才写入，记录的接收年龄 260.35 ms
含约 145 ms 清理时间，不能直接当成飞行时监督器停顿 260 ms。256 ms
测量年龄仍保留。现已在退出执行函数前记录 `execution_fault_detection`，
以后故障可区分门控检测与清理后的记录时间；这两轮旧失败不改写。

## 修改与边界

- 新增四段扩大、七段组合、横向专项三个路线 JSON，保留单目标 7 m 半径
  与每段 100 s 限制。
- 审计样本记录轨迹 id、起始时间、采样位姿时间，将实际发送 setpoint
  匹配回对应 B-spline。处理新轨迹 id 与上一轨迹位姿到达先后不同的情况。
- 五轮所有 TRACK 样本均成功匹配；未发现新的发布轨迹低于 0.55 m。
  样本匹配不能认证连续执行距离，也不撤销旧 29 m 运行的 0.453 m
  规划距离失败；旧样本缺少精确采样时间，不能补做确定归属。
- 63 项相关回归通过，见 `large_indoor_regressions_117.log`。
  原 250 ms 新鲜度、0.20 m 高度差、0.55 m 障碍距离等保护全部保留。
- 事件接收执行器仍显式实验启用，默认 `ready-batch`。事件模式没有 DDS
  MessageInfo，DDS 队列延迟不认证。飞行中未编译、离线分析、绘图或改执行源码。

下一步应针对长路线的调度/接收边界和高度残差分别修复及复测，不通过
反复完成较短路线宣称长时间可靠。

## 复现

在干净终端，每轮用新名称；所有仿真退出后才运行审计。

```bash
cd /home/cyf/dev/XT-Drone2-astra
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
 -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc -c \
 'bash astra/run_117.bash obstacle lateral_next --world-preset indoor-hall \
 --calibrate-ground-height --route-json astra/config/route_indoor_hall_lateral_only.json \
 --plane-quality-diagnostic --executor-audit --execution-executor events'

# 扩大四段：route_indoor_hall_large.json
# 七段组合：route_indoor_hall_large_lateral.json
# 回退接收器：--execution-executor ready-batch

python3 astra/tools/audit_indoor_hall.py astra/runtime/lateral_next
python3 astra/tools/audit_odom_delivery_117.py astra/runtime/lateral_next \
 --output astra/validation/lateral_next_delivery.json
python3 astra/tools/audit_executor_timing_117.py astra/runtime/lateral_next \
 --output astra/validation/lateral_next_executor.json
# 依赖上述 indoor_flight_audit.json，必须在 hall 审计完成后执行
python3 astra/tools/audit_spline_selection_117.py astra/runtime/lateral_next \
 --output astra/validation/lateral_next_selection.json
```
