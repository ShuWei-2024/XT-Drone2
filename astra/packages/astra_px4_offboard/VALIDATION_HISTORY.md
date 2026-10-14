# 历史坐标与执行桥实验

以下保留早期实验结论，不能作为当前运行教程。当前入口见 [模块说明](README.md)；最新融合和闭环状态见 [验证索引](../../validation/README.md)。

# Planner preview for Gazebo/SITL

Run `ros2 launch astra_px4_offboard preview_bspline.launch.py` alongside the
guarded map and EGO-Planner. The node samples `/planning/bspline` at 20 Hz and
publishes `/planning/position_preview` and `/planning/velocity_preview` in
`camera_init`. It checks map readiness, planner heartbeat, trajectory validity,
odometry freshness, trajectory time, and speed/acceleration limits. A fault
switches `/planning/preview_mode` from `TRACK_PREVIEW` to `HOLD_PREVIEW` or
`NO_VALID_HOLD`.

`/planning/px4_hold_preview` is a frozen position/yaw request in PX4 local
NED, captured from a fresh valid `/fmu/out/vehicle_local_position` at fault
entry. It is a diagnostic preview only. This node never publishes to
`/fmu/in/*`, never requests Offboard, and never arms PX4. The separate
`astra_offboard.launch.py` remains a hard-coded SITL baseline and does not
consume planner output.

`astra_px4_open.sdf` uses a PX4-controlled `astra_x500_px4` model with the
Mid-360-equivalent LiDAR, D435i-equivalent cameras, and IMU. The read-only
`audit_frame_alignment.py` compares Gazebo, FAST-LIVO2, and PX4 poses using
measurement timestamps, and checks translation, vertical position, body
heading, PX4 estimator resets, and heading quality. FAST-LIVO2 odom now uses
its LiDAR update time in the header; its timing CSV records publication time
separately.

For isolated Gazebo/PX4 runs using simulation time, pass `--px4-timebase sim`
when PX4 has `UXRCE_DDS_SYNCT=0`. Add `--samples-output path.csv` to retain
measurement-stamped Gazebo, FAST-LIVO2, and PX4 poses for phase-specific
residual and delay analysis. This mode bypasses the DDS wall-clock offset; it
does not certify a hardware clock or a planner execution bridge.

The latest moving audit paired all valid PX4 samples with Gazebo within 12 ms
and measured motion on both horizontal axes. The ENU/NED direction and heading
matched, with no estimator reset after in-flight heading alignment. PX4 XY and
Z fit residuals were 0.163 m and 0.170 m p95, above the 0.10 m gate. A
follow-up SITL route varied speed and direction and recorded all three pose
streams. A transform fitted on its outbound legs gave FAST-LIVO2 0.096 m XY
p95 on held-out return/diagonal legs, while PX4 remained at 0.134 m XY p95.
The transform is therefore still diagnostic, and the auditor did not certify
alignment. PX4 only sets `heading_good_for_control` after in-flight magnetic
alignment; ground samples with that flag false are excluded from the fitted
segment.

Stationary SITL ULogs show GPS altitude varying about 2.5 m over 15 seconds
while barometric altitude varied only about 0.005 m. A full PX4 restart with
barometric height as the reference did not by itself stabilize local Z.
Disabling GPS vertical aiding also did not suffice. With `EKF2_HGT_REF=0`,
`EKF2_GPS_CTRL=1`, and SITL-only `EKF2_BARO_NOISE=0.1`, local Z spanned
0.024 m and 0.032 m in consecutive 30-second stationary windows while
Gazebo was fixed. PX4 horizontal position still spanned up to 0.422 m in
those windows. Heading alignment, moving height accuracy, and long-run
stability remain unverified. The original PX4 values (`1`, `7`, and `3.5`)
were restored after the experiment. The horizontal transform is provisional
and specific to the earlier run; PX4 local origin can change after estimator
resets.

## Guarded B-spline bridge

Start `preview_bspline.launch.py`, then `guarded_bspline_bridge.launch.py`.
The bridge consumes the preview node's position, velocity, and mode plus
FAST-LIVO2 odometry and PX4 local position/status. It uses an explicit
`camera_init` to PX4 NED transform and checks frame and measurement-time
pairing, stale inputs, heading, estimator disagreement, speed, and tracking
error. The default launch publishes only `/planning/px4_setpoint_preview`
and `/planning/bridge_state`; it creates no `/fmu/in/*` publisher.

For an isolated, open-space PX4 SITL test only, launch with
`enable_px4_output:=true`. The bridge then streams position-mode heartbeat
and setpoints, but never arms PX4 or switches to Offboard. Those commands
require a separate operator-controlled SITL procedure after verifying the
diagnostic preview and frame transform. The `sitl_only` parameter is an
operator assertion, not a technical check that the connected PX4 is SITL.
The optional `freeze_tracking_yaw` parameter (default `false`, also exposed
as a launch argument) captures the PX4 heading at the first healthy tracking
update and keeps that yaw target across replans and successive goals. The
default mode continues to request the current estimated heading. Freezing
the target does not compensate for drift in the PX4 yaw estimate; estimator
reset and disagreement checks still apply. The isolated flight driver uses
the same frozen yaw during its final endpoint hold.

Once tracking has started, a gate failure freezes the current PX4 local pose
and latches the fault. If PX4 remains armed in Offboard after `max_hold_sec`
(default 1 second), the bridge requests land again every 0.5 second while
armed until PX4 enters Auto Land or disarms. Restart the node to clear a
latched fault. The default
transform is the previous diagnostic SITL fit and is not a certified hardware
calibration. Do not use this bridge for near-obstacle or hardware flight.
