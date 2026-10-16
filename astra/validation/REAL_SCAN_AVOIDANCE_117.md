# Real scanned room: goal-only closed-loop avoidance

Work on 2026-10-12 uses isolated PX4 1.17 SITL, Gazebo Harmonic, ROS 2 Jazzy,
FAST-LIVO2, live local mapping and EGO-Planner. No hardware or External Vision.
All downloaded/generated scene assets and raw runs remain in ignored runtime.

## Why the previous video did not demonstrate avoidance

The eight office0 legs in `replica_video_long_117_01` all had an unobstructed
direct route. Re-audit confirms ordinary motion, not a blocked-path detour.
Whole-flight known-surface clearance was 0.816 m. Office0's sampled full
descent region is concentrated near spawn; cruise space alone is insufficient
when recovery commands PX4 to land at current XY.

## Scene and bounded recovery

Inspected the pinned Replica-SLAM culled meshes office1, office2, office3,
office4, room0 and room2. Selected office4 without removing furniture or
rescaling the scan. Source revision:
`7d7ae90b48035aa2e9baa58018f5e090bd6d5dc5`.
`office4.ply` SHA256:
`dd2bbf6771825b2b3984408d32ca7f448f954420560f1c0d24c74a5e5b59d393`.
Preserve the Replica research license; this derivative has missing surfaces.

Native floor Z is -1.186634 m; native spawn XY is [1.284693, 2.761340] m.
World translation is [-1.284693, -2.761340, 1.156634] m, identity rotation,
unit scale. The invisible flat support floor remains synthetic. Visual and
concave collision geometry share the 100,000-face converted scan.

The recovery certificate samples scan distances at every corner/height of
0.05 m cells and subtracts the 3D grid covering radius. Only connected cells
with a continuous known-surface distance lower bound of at least 0.60 m are
allowed. The guard enforces both this cell union and the existing 0.55 m
measured distance gate, including during descent. A 0.6 s motion forecast
checks the complete forecast segment across the cell union, not just endpoints.
The inspected outer XY box is [-0.60,-2.20]..[2.90,0.50] m.
Body Z is [0.20,2.00] m. Scan holes remain uncertified.

## Fixes and isolated comparisons

1. PX4 requires in-flight magnetic alignment above 1.5 m. At 0.9 m cruise,
   `heading_good_for_control` never became true in run 01. Source evidence:
   `EKF/aid_sources/magnetometer/mag_control.cpp::checkHaglYawResetReq` and
   `EKF/ekf.h::isYawFinalAlignComplete`. First climb to 1.60 m at inspected
   spawn, await the existing heading flag/reset handling, then descend to
   0.90 m before giving goals. PX4 sources/parameters are not weakened.
2. Run 02 exhausted A*'s 200 ms limit repeatedly, starving its single-threaded
   planner heartbeat. The map was already inflated 0.65 m, while the optimizer
   requested another 0.8 m. Static scan experiments use 0.2 m optimizer
   clearance; the default remains 0.8 m. Actual clearance/freshness gates
   remain 0.55 m/250 ms. No extra heartbeat thread masks a stuck planner.
3. Run 03 exposed callback-order skew during height pairing. Retain an already
   matched fresh pair for at most 20 ms while a new corresponding PX4 callback
   arrives. The measurement-pair tolerance stays 20 ms and sample freshness
   stays 250 ms. Repeated unmatched frames cannot extend that wait indefinitely.
4. Run 04 reached the far goal, but returning replans violated scan distance
   in future portions. Static obstacle voxels expired after 3 s. Run 05 uses
   180 s retention and sparse live-map capture: all published splines passed
   known-surface clearance, but descent-region protection still stopped motion.
   This comparison is evidence for the retention change, not an exclusive
   diagnosis of every missing-obstacle case. Radius 5 m/capacity 5000 remain.
5. Runs 05/06 show that cruise clearance alone differs from safe descent.
   Run 07 optionally projects observed elevated furniture into vertical
   forbidden columns up to the planner ceiling. Floor and overhead ceiling
   are not extruded. Ordinary 3D occupied cells remain intact. The column
   policy is off by default and intended for this static-room recovery test;
   it is not dynamic-obstacle tracking or general free-space certification.

No prebuilt scan geometry, true pose or offline detour enters EGO's cloud or
setpoints. Geometry only chooses goals/test envelope and independently stops
unsafe SITL. Both goals are supplied directly; EGO chooses its own path.

## Reproduce

Keep an existing `REPLICA_LICENSE.txt` alongside the pinned downloaded source.
The converter verifies the SHA256 and refuses to overwrite scene directories.

```bash
cd /home/cyf/dev/XT-Drone2-astra
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_replica_office0.py \
  --scene office4 --source astra/runtime/avoidance_20261012/office4.ply \
  --output astra/runtime/maps/replica_office4
python3 astra/tools/prepare_scan_avoidance.py \
  --scene-dir astra/runtime/maps/replica_office4 --goal-xy 2.4 -1.1 \
  --xy-bounds -0.60 -2.20 2.90 0.50 \
  --takeoff-height-m 0.9 --alignment-height-m 1.60
```

Use a new run name each time. The runner verifies route and asset hashes,
snapshots execution sources, isolates DDS/Gazebo, preserves original baseline
files and cleans up owned processes after landing/disarming.

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc -c \
  'bash astra/run_117.bash obstacle replica_office4_avoidance_117_07 \
   --world-preset replica-scan --scan-dir astra/runtime/maps/replica_office4 \
   --route-json astra/runtime/maps/replica_office4/route.json \
   --calibrate-ground-height --executor-audit'
python3 astra/tools/audit_scan_flight.py \
  astra/runtime/replica_office4_avoidance_117_07
```

The auditor freezes frame offsets from ground measurements, checks measured
movement/endpoints, measures lateral detour from a blocked direct path, checks
whole-flight including recovery distances/envelopes, transition holds,
combined setpoint timing and planned derivative bounds. Spline clearance uses
sampled distance minus a convex-hull speed bound times half the sample gap;
uncertain cases refine to 5 ms rather than declaring samples sufficient.

Detailed run results are recorded in `real_scan_avoidance_117.json`.
Earlier failures are retained. A complete nominal run is not a repeatability,
infinite-runtime, unobserved-geometry or hardware-flight certificate.

## Completed route and repeat

Runs `replica_office4_avoidance_117_07` and `_08` both completed the goal-only
route `[[2.4,-1.1],[0,0]]`, then landed and disarmed. The final strict auditor
passed both complete-route and blocked-direct-path detour checks. Neither
flight triggered a failsafe; baseline files and execution sources were
preserved. No External Vision fusion was active.

| Measurement | Run 07 | Run 08 |
| --- | ---: | ---: |
| Actual measurement-paired horizontal path | 7.319 m | 7.403 m |
| Minimum whole-flight known-surface distance | 0.871 m | 0.775 m |
| Minimum planned continuous distance lower bound | 0.833 m | 0.777 m |
| Outbound / return endpoint error | 0.028 / 0.025 m | 0.075 / 0.044 m |
| Outbound / return lateral detour | 0.702 / 0.935 m | 0.658 / 0.848 m |
| Maximum combined setpoint transmission gap | 66.1 ms | 69.9 ms |
| Published B-splines | 119 | 123 |

Direct segments approach known scan surfaces within 0.44–0.47 m, below the
0.55 m test margin. Measured paths take a lateral detour and meet the margin.
All published spline derivative bounds met the 0.10 m/s and 0.10 m/s²
configuration with 0.001 numerical tolerance. Those are planned constraints;
actual tracking motion is recorded separately. Transition holds and the
whole-flight recovery envelope passed.

Distances use the `base_link` reference point and retained scan surfaces,
excluding the floor; they do not certify clearance of every rotor or missing
surface. Two repetitions establish evidence for this route only. They do not
establish general dynamic avoidance or long-term reliability.

Audits and plots remain in each ignored runtime directory:
`scan_flight_audit.json` and `scan_actual_path.png`. The tracked JSON records
compact results and the earlier failures without redistributing scan assets.

## Visible sensor exterior

After both flight runs, added a silver housing, dark cap, mounting foot and
front badge to `astra_x500_sensors` as a stylized Mid-360 exterior. These are
visual elements only; mass, inertia, collisions, sensor poses and extrinsics
remain identical. Housing surfaces lie below the lowest LiDAR scan ray.
Appearance validation is separate from the above flight evidence.

Passive run `replica_office4_radar_visual_117_01` passed 25 seconds of sensor
checks after this change: every captured cloud had 11,520 finite points,
readiness was 100%, odom ran at 9.95 Hz and its measurement age p95 was 32 ms.
The first raw cloud's minimum range was 1.090 m, with no returns below 0.25 m.
All expanded geometry/extrinsic checks passed. Removing the four new visuals
from the copied model produces the same nonvisual XML as flight run 08.
These passive checks were followed by the recorded flights below.

## Recorded exterior flights

Both `replica_office4_avoidance_video_117_01` and `_02` completed the same
guarded route with the new exterior, Gazebo GUI and NVENC recording active.
The strict complete-route, blocked-path, planned dynamics and continuous
clearance checks passed. No failsafe occurred. The second recording uses a
fixed elevated camera from startup so the ground vehicle is visible;
the initial climb briefly approaches the right image edge.

Final video: `astra/runtime/recordings/Replica-office4-PX4-1.17-closed-loop-avoidance.mp4`.
It removes the first 40 seconds of startup waiting, preserves original speed
and includes takeoff, goal-only detour, return and landing. Run 02 travelled
7.355 m horizontally, with a minimum whole-flight known-surface distance of
0.829 m and endpoint errors of 0.018 / 0.026 m. Setpoint gaps stayed below
63.5 ms. GUI/recording load means this is not a headless timing comparison.
Raw captures, recording commands and logs remain in ignored runtime.
