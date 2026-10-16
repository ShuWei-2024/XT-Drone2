# 解锁前 EV／全局原点初始化中止

2026-10-16，用户运行 `outdoor_20261016_171332` 在初始化阶段退出，尚未进入选点或发送解锁命令。本事件与航点可选区域扩大无关；保留原运行和失败摘要，不覆盖日志。

## 原运行证据

| 事件 | 证据 |
| --- | --- |
| EV 发布 | `external_vision.log` 记录地面参考冻结并达到 ready |
| 原点设置命令 | `SET_GPS_GLOBAL_ORIGIN`；ACK 在 PX4 38.340 s 返回 ACCEPTED（result=0） |
| PX4 原点实际生效 | ULog 38.344 s：`ref_alt=0.33757105`，全局高度 `alt_valid=1`，经纬度有效 |
| 局部估计 | 同时 local `(−0.029487,0.023312,0.097943)` m；XY/Z reset counter 因设置原点从 4/2 变为 5/3，航向计数保持 1 |
| 初始化程序退出原因 | `stale or future measured position` |
| 原参数／源码 | `run_summary.json` 确认原基线保持，源码未在该轮运行中改变 |

ACK 接受和高度生效，不等于后续连续健康确认已经完成。旧程序第一阶段能等待有效数据，发命令后的第二阶段却在任何一次过期／未来时间戳回调上直接抛异常退出，哪怕下一帧可以恢复。

原报告没有保存失败时本节点时钟、测量年龄与接收年龄。因此只能确认是哪条检查触发，不能进一步断言是处理积压、DDS 传输延迟或 `/clock` 回调排队导致；PX4 ULog 中原点附近的本地／全局位置仍持续更新，也不足以反推接收端年龄。不能说 EV 没融合、原点没设上或飞机飞行崩溃。

## 修改

`TelemetryPending` 仅标识暂时缺少／过期／超前的遥测；此时保持地面等待、清零连续健康计时，再取后续输入。原总期限 60 秒、local 250 ms、GNSS 500 ms、接收年龄上限和连续健康 1 秒不变，不插值、不重新标记时间、不重发原点命令。

解锁、EV XY/Z 融合停止、非有限输入、GNSS 质量不合格或非静止状态仍拒绝，命令后的局部位置变化超过 0.03 m 仍中止。持续无有效数据仍超时，不能因为收到 ACK 就绕过确认。

报告新增有界的 pending 计数／输入快照（最多前 16 条及最后一条）和最终失败快照，记录本节点仿真时间、local／GNSS 测量年龄、各话题接收年龄；启动器直接显示具体失败原因。

289 项回归通过（3 个既有弃用警告）。针对性回归覆盖临时数据问题后完整重新稳定、持续缺失仍在原期限内失败、只发送一次命令，以及真实健康故障不被当作临时问题忽略。

## 地面启动复测

`outdoor_ev_origin_startup_20261016_01` 初始化确认通过、被动健康检查通过，进入选点界面；仅检查地面启动，不请求开始任务、不执行飞行。

本轮实际触发一次 pending 分支：节点时钟 **34.780 s**，local 测量年龄 **8 ms**，GNSS 测量年龄 **−4 ms**（样本 34.784 s，刚收到约 0.064 ms）。这证明本节点仿真时钟与遥测短暂不同步，会命中旧程序的未来样本拒绝；不能单凭这份记录把更底层原因限定为 DDS、调度或 `/clock` 回调其中之一。

修复没有接受这个未来样本，而是重启连续健康计时；在 **35.796 s** 确认通过。只发了一次原点命令，ACK 为 ACCEPTED，局部位置前后差约 **0.0091 m**，仍小于原 0.03 m 限制；EV／GNSS 条件恢复后才继续。由此本轮不只是“没有再碰到问题”，而是实际走到了修复分支。原失败轮缺少年龄日志，不能说它必定也正好超前 4 ms。

```bash
bash astra/rviz_outdoor_117.bash outdoor_ev_origin_startup_NEW \
  --external-vision --rviz-goal-timeout-sec 30
```

30 秒是本次测试的地面选点等待期，未点击开始时将以 `RViz mission editing timed out without arming` 结束，这是有意的测试收尾，不算初始化失败或完成飞行。日常使用省略 `--rviz-goal-timeout-sec 30`，地面继续不限时等待。

```bash
source astra/env_117.bash
python3 -m pytest -q astra/validation astra/packages/astra_px4_offboard/test astra/packages/astra_fast_livo2/test
```

本机回归日志 `astra/runtime/ev_origin_pending_regression_20261016.log`。可随仓库获取的 [结构化证据](ev_origin_startup_117_evidence.json)；完整 ULog 和启动记录位于忽略的 `astra/runtime/`。本次修复不证明底层调度／DDS 抖动已消除，也不新增飞行或跨地图认证。
