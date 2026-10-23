# PX4 1.17 高度耦合隔离验证

日期：2026-10-21。测试使用原有 1.17 SITL、FAST-LIVO2 External Vision、3 Pa 气压噪声和原高度保护阈值；没有放宽保护，也没有连接真实硬件。

## 测试命令

```bash
bash astra/run_117.bash small horizontal_height_isolation_20261021_01 \
  --world-preset indoor-hall --external-vision --calibrate-ground-height --executor-audit

bash astra/run_117.bash small vertical_height_isolation_20261021_01 \
  --world-preset indoor-hall --external-vision --calibrate-ground-height --executor-audit \
  --vertical-isolation

bash astra/run_117.bash obstacle horizontal_rectangle_isolation_20261021_01 \
  --world-preset indoor-hall --route-json astra/config/route_lateral_rectangle.json \
  --external-vision --calibrate-ground-height --executor-audit --end-action land

bash astra/run_117.bash obstacle horizontal_rectangle_lockz_20261021_01 \
  --world-preset indoor-hall --route-json astra/config/route_lateral_rectangle.json \
  --external-vision --calibrate-ground-height --executor-audit --end-action land \
  --lock-planner-height
```

## 结果

| 隔离项 | 结果 |
|---|---|
| 固定高度水平前进/侧移 | LIVO+PX4 高度残差 p95 约 0.020 m；没有持续累积 |
| 纯垂直上升/下降 | LIVO+PX4 残差 p95 约 0.039 m；回到原高度后约 0.011 m |
| 长矩形路线，默认规划器 | 目标高度跨度 0.319 m；实际高度跟踪误差 p95 0.071 m，最大 0.177 m |
| 长矩形路线，锁定规划高度 | 规划器/PX4 内部目标高度跨度 0 m；实际真值高度仍波动约 0.254 m，最大跟踪误差 0.222 m |

## 判断

EGO-Planner 的重规划起点直接使用 FAST-LIVO2 当前三维 odom。平面航线的目标 Z 虽固定，但 LIVO Z 的小变化会进入新 B-spline，进而改变 PX4 的垂直 setpoint。`--lock-planner-height` 已验证可以消除这一级放大，但它没有解决 PX4 估计值和 Gazebo 真值之间的动态高度误差。

因此当前问题分成两层：

1. **规划器耦合**：已确认，已有可配置诊断开关，默认关闭；
2. **飞控/估计器动态高度误差**：仍需单独处理，不能通过放宽高度保护解决。

当前不应据此认证近障碍飞行或真实硬件。下一步应在锁定高度诊断模式下继续检查 PX4 的 `vehicle_local_position`、External Vision height innovation、垂直速度和 Gazebo 真值，确认剩余波动是控制器过冲还是 EKF 共同偏差。
