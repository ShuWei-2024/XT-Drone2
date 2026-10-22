# PX4 1.17 垂直 GNSS 速度融合隔离测试

日期：2026-10-21。测试在隔离 SITL 中进行，使用相同室内矩形路线、FAST-LIVO2 External Vision、锁定规划高度和默认 3 Pa 气压噪声。没有修改默认 PX4 参数，也没有放宽保护。

## 测试目的

判断 PX4 融合的 GNSS 三维速度是否是水平飞行时高度偏差的重要来源。PX4 1.17 的 `EKF2_GPS_CTRL` 将三轴速度作为同一位；无法只关闭垂直速度。因此诊断值从 `5` 改为 `1`，保留 GNSS 水平位置，关闭三轴 GNSS 速度融合。

## 测试命令

```bash
bash astra/run_117.bash obstacle height_lockz_nognssvel_20261021_01 \
  --world-preset indoor-hall --route-json astra/config/route_lateral_rectangle.json \
  --external-vision --calibrate-ground-height --executor-audit --end-action land \
  --lock-planner-height --disable-gps-velocity-fusion

bash astra/run_117.bash obstacle height_lockz_nognssvel_repeat_20261021_01 \
  --world-preset indoor-hall --route-json astra/config/route_lateral_rectangle.json \
  --external-vision --calibrate-ground-height --executor-audit --end-action land \
  --lock-planner-height --disable-gps-velocity-fusion
```

两次任务均通过 preflight、完成路线并安全解除武装。ULog 确认 `EKF2_GPS_CTRL=1` 且 GNSS velocity fusion 样本数为 0；气压高度和 External Vision 高度仍在融合。

## 结果

| 配置 | GNSS 速度融合 | 高度误差 p95 | 最大绝对跟踪误差 | PX4 高度估计误差 p95 |
|---|---:|---:|---:|---:|
| 默认锁高基线 | 开 | 0.046 m | 0.222 m | 0.040 m |
| 3 Pa，关闭 GNSS 速度融合 | 关 | 0.073 m | 0.136 m | 0.074 m |
| 重复运行 | 关 | 0.072 m | 0.208 m | 0.057 m |

## 判断

两次关闭 GNSS 速度融合都没有稳定降低高度误差：最大误差分别为 0.136 m 和 0.208 m，p95 分别为 0.073 m 和 0.072 m；默认基线的一次最大值为 0.222 m、p95 为 0.046 m。测试重复性不足以证明 GNSS 速度融合是主要原因，也不能仅凭三次运行比较宣称关闭它改善或恶化了系统。此项只排除了“关闭 GNSS 速度即可可靠消除瞬态误差”的简单假设。

低噪声气压对照（0.05 Pa）确实把原始气压散布从约 0.281 m 降至约 0.006 m，但最大绝对跟踪误差仍为 0.188 m，说明气压噪声也不是唯一解释。

下一步应把每次航段按相同阶段对齐，比较垂直加速度/推力响应、PX4 估计速度和 Gazebo 真值速度，并重复默认配置建立基线。当前不更改默认 `EKF2_GPS_CTRL=5`，不放宽保护；此高度表现不能作为近障碍或真实硬件认证。

## 高度峰值动态复核（2026-10-21）

对 `height_lockz_default_repeat_20261021_02` 和 `_03` 的已完成飞行重新按测量时间分析，比较 Gazebo 真值、PX4 `vehicle_local_position`、PX4 内部位置控制输出和估计器创新。审计脚本同时保存每个峰值前后 `-2、-1.5、-1、-0.5、0、0.5、1、2 s` 的快照：

```bash
python3 astra/tools/audit_height_dynamics_117.py \
  astra/runtime/height_lockz_default_repeat_20261021_03 \
  --output astra/runtime/height_lockz_default_repeat_20261021_03/height_dynamics_audit.json \
  --plot astra/runtime/height_lockz_default_repeat_20261021_03/height_dynamics_audit.png
```

两个重复测试的峰值都有相同的动态形态：目标高度没有变化；峰值前约 1 秒 Gazebo 真值出现约 `0.076~0.093 m/s` 的向下速度，而 PX4 估计的向上速度仍接近 `-0.015~0.024 m/s`；峰值时真值速度已接近零，PX4 才估出约 `0.028~0.041 m/s` 的向上速度。对应峰值处 PX4 高度估计偏差约 `+0.058~+0.071 m`，控制器估计跟踪误差约 `-0.040~-0.050 m`，实际高度误差约 `-0.102~-0.110 m`。

这支持“垂直速度/高度估计相对实际下沉存在滞后，并叠加控制回正误差”的工作假设；它没有证明具体是气压计、GNSS 速度、External Vision 延迟或控制器参数中的哪一项。气压、GNSS 垂直速度和 EV 高度创新在窗口内均有变化，但没有出现明确的融合拒绝或滤波故障。当前不据此修改 PX4 参数，也不放宽高度保护；下一步若需定位根因，应做单因素时间延迟或传感器输入隔离实验。

补充的局部时间偏移搜索也没有得到一个统一延迟：高度曲线的最佳偏移约为 `+0.35～+0.50 s`，而垂直速度曲线的最佳偏移在 `-0.325～-0.075 s` 之间。这个方向和幅度不一致，不能把保护阈值简单放宽 `0.3～0.5 s`，也不能把它写成“PX4 固定延迟”。它更符合滤波、传感器融合和位置控制响应叠加后的动态差异。

## PX4 输出预测器线索

两次 ULog 的参数均为 `EKF2_TAU_POS=0.25` 和 `EKF2_TAU_VEL=0.25`。PX4 1.17 的 EKF output predictor 会根据这两个时间常数，把延迟的 EKF 状态与高频 IMU 输出做互补校正；它不是传感器时间戳延迟参数，但会影响发布给位置控制器的 `vehicle_local_position` 的相位和瞬态响应。源码位置为：

```text
PX4-Autopilot-1.17/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp
```

源码中的位置、速度校正增益分别使用 `dt / EKF2_TAU_POS` 和 `dt / EKF2_TAU_VEL`。因此当前证据链可以表述为：

```text
实际模型先发生小幅下沉
→ EKF/传感器融合与 output predictor 逐步修正状态
→ 位置控制器随后产生回正指令
```

这解释了动态现象的一个合理机制，但不能单独证明应当修改 `EKF2_TAU_*`。下一步若要验证，应在隔离 SITL 中只改变一个 `EKF2_TAU_POS/VEL`，比较峰值误差、稳定性和融合状态；在此之前保持默认参数。

## Output predictor 单因素对照

为验证上述线索，在相同 `indoor-hall` 矩形路线、External Vision、锁定规划高度和原有保护条件下，分别运行了两个隔离 SITL。每次只覆盖一个 PX4 参数，运行结束均安全降落，`original_baseline_preserved=true`。

| 配置 | 实际高度误差 p95 | 最大负误差 | `>0.10 m` 峰值 | PX4 估计误差 p95 |
|---|---:|---:|---:|---:|
| 默认 `0.25/0.25`（重复 `_03`） | `0.0482 m` | `-0.1100 m` | 2 | `0.0404 m` |
| `TAU_POS=0.10`，`TAU_VEL=0.25` | `0.0576 m` | `-0.0916 m` | 0 | `0.0565 m` |
| `TAU_POS=0.25`，`TAU_VEL=0.10` | `0.0438 m` | `-0.1130 m` | 1 | `0.0506 m` |

对应运行目录：

```text
astra/runtime/height_lockz_taupos010_20261021_01
astra/runtime/height_lockz_tauvel010_20261021_01
```

结果显示明显权衡：降低 `TAU_POS` 消除了本次路线中的 `>0.10 m` 峰值，但整体 p95 和 PX4 估计误差上升；降低 `TAU_VEL` 略微改善 p95，却放大了最大瞬态误差。单次对照不能证明哪一个更适合所有路线，因此保持默认 `0.25/0.25`，不把诊断值写入正式配置。
