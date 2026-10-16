# 历史集成与诊断记录

本文件保存早期分阶段适配和延迟实验；其中绝对路径、构建 overlay 与旧默认值仅对应当时实验。当前使用请读 [模块说明](README.md) 和 [总文档索引](../../../docs/README.md)。

# astra_fast_livo2

This package is the ROS 2 Jazzy input boundary for FAST-LIVO2.
The public FAST-LIVO2 reference at `/home/cyf/dev/reference/FAST-LIVO2` is
revision `0d2c0346107b75b59934975adec9a6eeeb913c64` and remains a ROS 1
catkin project. Its `livox_ros_driver/CustomMsg` contract was re-expressed in
`astra_fast_livo2_msgs`; no ROS 1 headers or catkin files are compiled here.
The ROS 2 estimator now runs from the separate `astra_fast_livo2_core` ament
package. The older investigation of `freshleesh/fast_livo2` is superseded by
the fourth-stage implementation described below.

## Stage 3 input adapter

`astra_fast_livo2_adapter` subscribes to RGB, PointCloud2, CameraInfo and IMU,
checks frame IDs and monotonic timestamps, measures nearest LiDAR/RGB and
LiDAR/IMU time differences, and republishes normalized inputs. It does not
perform LiDAR-inertial-visual odometry and it does not publish an estimated
pose. The separate C++ estimator publishes the fourth-stage pose outputs.
No Gazebo truth odometry is relabeled as FAST-LIVO2.

The output `/fast_livo2/input/livox_custom` is a ROS 2 equivalent message. The
Gazebo lidar has no physical Livox per-point timing, so `offset_time` is a
monotonic whole-frame approximation over `scan_period_sec=0.1`; this is
explicitly simulation-only and does not reproduce the Mid-360 firing pattern.
The enriched PointCloud2 output contains `x,y,z,intensity,time,ring`, with
`time` in microseconds as required by the reference standard-cloud handler,
and `ring` in the range 0..15. The Livox-equivalent `offset_time` is in
nanoseconds.

## Run

In separate terminals, start the stage-2 sensor launch and the adapter:

```bash
source /opt/ros/jazzy/setup.bash
source /home/cyf/dev/astra_ros2_ws/install/setup.bash
ros2 launch astra_gz_sensors gz_sensors.launch.py rviz:=false
```

```bash
source /opt/ros/jazzy/setup.bash
source /home/cyf/dev/astra_ros2_ws/install/setup.bash
source /home/cyf/dev/astra_ros2_ws/install_phase3/setup.bash
ros2 launch astra_fast_livo2 fast_livo2_adapter.launch.py
```

Build only the phase-3 packages into separate directories:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/cyf/dev/astra_ros2_ws
colcon --log-base log_phase3 build \
  --packages-select astra_fast_livo2_msgs astra_fast_livo2 \
  --symlink-install --build-base build_phase3 --install-base install_phase3
```

The adapter uses best-effort sensor-data QoS for all sensor subscriptions and
sensor outputs, and reliable QoS for diagnostics. With the current Python
conversion and Gazebo rendering load, delivered RGB and IMU rates can drop
below their nominal 30 Hz and 100 Hz; inspect `/fast_livo2/diagnostics` before
running any estimator. Synchronization warnings indicate the current input
stream is not yet qualified for odometry.

Adapter topics:

| Gazebo/bridge input | Adapter output | Type |
| --- | --- | --- |
| `/sensors/lidar/points` | `/fast_livo2/input/lidar` | `sensor_msgs/msg/PointCloud2` |
| `/sensors/lidar/points` | `/fast_livo2/input/livox_custom` | `astra_fast_livo2_msgs/msg/LivoxCustomMsg` |
| `/sensors/camera/color/image_raw` | `/fast_livo2/input/image` | `sensor_msgs/msg/Image` (`rgb8`) |
| `/sensors/camera/color/camera_info` | `/fast_livo2/input/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/sensors/imu/data` | `/fast_livo2/input/imu` | `sensor_msgs/msg/Imu` |
| - | `/fast_livo2/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

The camera calibration in `config/fast_livo2_gazebo.yaml` comes from live
Gazebo CameraInfo: 640x480, `plumb_bob`, `fx=462.26634979`,
`fy=462.26637840`, `cx=320`, `cy=240`, zero distortion, identity rectification,
and the matching projection matrix. Static TF-derived extrinsics are
LiDAR-to-IMU translation `[0, 0, -0.06]` with identity rotation, and
LiDAR-to-camera-optical translation `[0, -0.12, -0.16]` with rotation
`[[0,-1,0], [0,0,-1], [1,0,0]]`. These are simulated mounts, not physical
sensor calibration.

In a 10-second local probe, the outputs contained 101 LiDAR scans, 102 Livox
equivalent scans, 198 RGB images and 695 IMU samples. The custom message had
5525 valid points in the sampled scan, with strictly increasing synthetic
offsets from 0 to 99,331,539 ns and lines 0..15. Diagnostics reported timing
threshold violations under load. These measurements verify adaptation, not
estimator readiness. The `/fast_livo2/diagnostics` values are the current
source of rate and nearest timestamp differences; configured limits are 60 ms
for LiDAR/RGB and 20 ms for LiDAR/IMU.

The measurements above describe the earlier adapter-only stage. The estimator
has since been built and produced all four requested outputs; see below.

## Stage 4: FAST-LIVO2 C++ algorithm

The algorithm package `astra_fast_livo2_core` comes from
`U-AMC/FAST-LIVO2-ROS2` revision
`a09599a288c56b832114d7ad9e898248a4111d9e` and was adapted locally for
Jazzy. The ROS 1 algorithm at `/home/cyf/dev/reference/FAST-LIVO2` revision
`0d2c0346107b75b59934975adec9a6eeeb913c64` remains a reference only.
`vikit_common` and `vikit_ros` come from the Jazzy branch of
`Lmh-sys/rpg_vikit_ros2` revision
`5b78f6536366182351fd5b95d1506340188dccf7`. Sophus legacy revision
`a621ff` is built privately at `build_sophus_legacy`; its six tests pass.
Two of three `vikit_common` tests pass. `test_vk_common_camera` attempts an
approximately 128 GB allocation and fails on this 15 GiB host.

The core is a real ament C++ algorithm build using Eigen, PCL, OpenCV,
cv_bridge, image_transport, ROS 2 messages and tf2. It uses the standard
`PointCloud2` preprocessing path (`lidar_type=2`, `scan_line=16`). The local
Livox-equivalent ROS 2 message supports adapter tests, but the combined
launch disables its publication. No physical Livox driver is needed.

Build from `/home/cyf/dev/astra_ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon --log-base log_phase3 build \
  --packages-select astra_fast_livo2_msgs astra_fast_livo2 \
  --symlink-install --build-base build_phase3 --install-base install_phase3
source install_phase3/setup.bash
colcon --log-base log_phase4 build \
  --packages-select astra_fast_livo2_core \
  --symlink-install --build-base build_phase4 --install-base install_phase4 \
  --cmake-args -DSophus_DIR=/home/cyf/dev/astra_ros2_ws/build_sophus_legacy \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_PARALLEL_LEVEL=2
```

The existing `install/` and `install_phase4/` overlays contain
`astra_gz_sensors`, `vikit_common` and `vikit_ros`. In each run terminal,
source `/opt/ros/jazzy/setup.bash`, `install/setup.bash`,
`install_phase3/setup.bash`, then `install_phase4/setup.bash`. Start:

```bash
ros2 launch astra_gz_sensors gz_sensors.launch.py rviz:=false
```

```bash
export LD_LIBRARY_PATH=/home/cyf/dev/astra_ros2_ws/build_sophus_legacy:$LD_LIBRARY_PATH
ros2 launch astra_fast_livo2 fast_livo2.launch.py
```

The combined launch starts a `/clock` bridge, the adapter and the C++ core.
Adapter and core both use `use_sim_time=true`. Sensor subscriptions and
adapted sensor publications use best-effort sensor-data QoS, except for the
combined launch's reliable RGB subscription. Diagnostics and
algorithm pose, path and registered-cloud publishers use reliable QoS. The
sensor launch uses `parameter_bridge`; the former `bridge_node` crashed during
dynamic-library unloading on SIGINT, whereas `parameter_bridge` exited cleanly.

| Stable Gazebo input | Adapted core input | Type |
| --- | --- | --- |
| `/sensors/lidar/points` | `/fast_livo2/input/lidar` | `sensor_msgs/msg/PointCloud2` |
| `/sensors/camera/color/image_raw` | `/fast_livo2/input/image` | `sensor_msgs/msg/Image` (`rgb8`) |
| `/sensors/camera/color/camera_info` | `/fast_livo2/input/camera_info` (adapter check) | `sensor_msgs/msg/CameraInfo` |
| `/sensors/imu/data` | `/fast_livo2/input/imu` | `sensor_msgs/msg/Imu` |

The estimator itself publishes `/fast_livo2/odom` (`nav_msgs/msg/Odometry`),
`/fast_livo2/path` (`nav_msgs/msg/Path`), `/fast_livo2/vision_pose`
(`geometry_msgs/msg/PoseStamped`) and `/cloud_registered`
(`sensor_msgs/msg/PointCloud2`). `/fast_livo2/diagnostics` reports adapter
input health, not estimator health. The estimator frame `camera_init` is its
own startup frame, not Gazebo truth `map` or `odom`.

The core config is `astra_fast_livo2_core/config/fast_livo2_gazebo.yaml`.
Gazebo CameraInfo is 640x480, `plumb_bob`, `fx=462.26634979`,
`fy=462.26637840`, `cx=320`, `cy=240`, zero distortion. Simulated TF gives
LiDAR-to-IMU translation `[0, 0, -0.06]`, identity rotation; LiDAR-to-camera
optical translation `[0, -0.12, -0.16]`, rotation
`[[0,-1,0], [0,0,-1], [1,0,0]]`. These are simulated mounts, not physical
sensor calibration. The Gazebo sensor is a 16-ring Mid-360 *equivalent* at
nominal 10 Hz. Point `time` is synthesized across the 0.1 s scan in
microseconds. It does not reproduce real Mid-360 firing times or point order.
Depth is not an algorithm input.

## Current stability measurements

Relevant changes are in `astra_fast_livo2_core` (executor, bounded queues,
Eigen/PCL alignment, VIO point lifetime), `astra_fast_livo2` (bounded input
pairing, diagnostics and trajectory probes), and `astra_gz_sensors` (the
`parameter_bridge` launch and separate motion model/world). The static
sensor model and world remain available under their original names.

The adapter keeps bounded LiDAR (8), RGB (24) and IMU (400) queues. LiDAR is
the pairing clock; the nearest RGB stamp must be within 30 ms by default, and
the scan must have an IMU window. Rejected pairs are counted by reason on
`/fast_livo2/diagnostics`. IMU is forwarded continuously to the core even
when a LiDAR/RGB pair is rejected. The core has separate executor and mapping
threads, bounded input queues, and callback, processing, publication and queue
drop counters in its logs. Raw bridge delivery is approximately 10 Hz LiDAR,
30 Hz RGB and 100 Hz IMU; the adapter sees about 27-29 Hz RGB in the static
world. An earlier 2,214-scan static run paired 2,169 scans; 44 exceeded the
RGB limit, none lacked IMU, and no queue overflow occurred. Successful pairs
had mean RGB offset 8.74 ms, P95 20 ms, maximum 28 ms. An external probe
subscribing to full images and clouds can itself reduce throughput, so these
pairing figures come from adapter-internal counters.

The four-minute stationary probe after visual-map proximity filtering recorded
2,180 odometry and path messages, 2,179 vision poses and 4,349 registered
cloud messages. Odom ran at 9.12 Hz, with no non-finite pose and maximum
adjacent translation 0.4 mm. Core RSS peaked at 199,556 KiB (about 195 MiB)
and was about 197 MiB at four minutes; before filtering, it had exceeded
1 GiB at four minutes. Each 0.5 m visual voxel now rejects a new point within
5 cm of an existing point, deleting its owned image feature. This bounds
growth in the tested stationary scene, not in all trajectories. The core,
adapter and clock bridge each exited with code 0 on SIGINT.

To run the separate motion validation scene, start the sensor launch with
`world:=astra_motion.sdf`. It uses a gravity-disabled velocity-controlled
body with the same sensor mounts and fixed landmarks; it does not control PX4
or use Gazebo truth as an estimator input. After starting the combined
FAST-LIVO2 launch, publish `gz.msgs.Twist` to the model, for example:

```bash
gz topic -t /model/astra_x500_motion/cmd_vel -m gz.msgs.Twist -p 'angular: {z: 0.05}'
```

Publish a zero twist to stop. A 0.05 rad/s yaw command produced about
0.04994 rad/s on IMU z. In a forward, lateral and yaw sequence, an independent
trajectory probe recorded 652 odometry messages over 110 s (5.93 Hz), no
non-finite poses and maximum adjacent translation 0.159 m. After stopping,
Gazebo pose was about (2.289, 2.308) m, yaw 0.467 rad; the estimator was
about (2.264, 2.284) m relative to its own startup frame, yaw 0.486 rad.
These snapshots are a motion sanity check, not an accuracy certificate:
startup frames differ and no time-aligned ground-truth error series was
computed. The motion launch publishes sensor mount transforms but omits the
static world-to-body TF chain, since that chain would misrepresent a moving
body; it does not yet provide a dynamic body TF. Moving-run diagnostics at
2,323 LiDAR scans showed 1,920 pairs,
400 RGB offset rejections, two missing IMU windows and no queue overflow.
Successful RGB offsets had mean 11.04 ms, P95 28 ms and maximum 28 ms.
Core RSS reached about 488 MiB after five minutes, then increased slowly.
Moving-run core, adapter and clock bridge also exited with code 0 on SIGINT.

### Moving endurance and memory

The combined launch exposes `max_lidar_image_delta_sec` (default `0.03`) and
`max_total_visual_points` (default `5000`). The adapter converts PointCloud2
fields with NumPy and skips an unused unsynchronized RGB republish in this
launch. It uses a reliable image subscription to match the Gazebo bridge's
reliable RGB publisher. The adapter-only launch retains a best-effort image
subscription for other input sources. The motion model's unused depth camera
runs at 5 Hz; RGB remains configured at 30 Hz. The published path retains at
most 2,000 poses. The
Gazebo profile limits visual-map points to eight per 0.5 m voxel and 5,000
overall; the overall limit evicts least recently observed points between
frames. The generic core defaults remain unlimited. Core logs every five
seconds include LiDAR and visual voxel counts, visual points, observations,
distinct retained images and their size, rejected/pruned points, queue drops,
and processing counts.

Use `scripts/measure_endurance.py CSV --seconds 900 --pid CORE_PID --drive`
after starting the motion world and combined launch. It repeatedly commands
slow translation and yaw, samples estimator RSS every 30 seconds, and checks
finite `/fast_livo2/odom`, nonempty `/cloud_registered`, timestamp order and
maximum receive gaps. It commands zero velocity on exit. The `--drive` scene
is a simulated test body, not PX4 or a flight controller. Add `--wide` to
command the nominal 4 m square instead of the small loop; a landmark blocks
the positive-y leg near y=0.975 m, so this is a collision reproduction route,
not a completed 4 m square. The driver now repeats each velocity command
through one persistent Gazebo Transport publisher. For a guarded test, set
`--max-odom-step-m 0.5 --max-odom-displacement-m 20 --max-rss-kib 2000000`;
the probe then stops motion and records the final sample on an exceeded limit.
The sensor and estimator launches must still be stopped separately.

A 10-minute moving baseline without the visual-point budget reached
1,430,568 KiB core RSS and was still rising. It produced 3,836 finite odometry
messages and 3,837 nonempty clouds; 691 of 6,231 LiDAR scans failed the 30 ms
RGB limit. Limiting to eight points per visual voxel reduced a second
10-minute peak to 1,123,056 KiB but did not stop growth. Full grayscale
images held by visual observations account for much of the memory increase.

With the 5,000-point total budget, a 15-minute 30 ms run produced 5,359
finite odometry messages and 5,359 nonempty clouds. RSS peaked at 921,984 KiB
and stayed near that level during the final minutes; over 7,000 visual points
were pruned. The maximum odometry receive gap was 1.217 s. Synchronization
still failed 1,118 of 9,094 scans on the RGB limit, with one missing IMU
window and four adapter queue overflows. RGB delivery fell to about 18.7 Hz.
Repeated-route RSS stability does not establish a bound on a larger route:
LiDAR voxel count can still grow with explored area.

A separate three-minute 50 ms trial paired 1,982 of 2,009 scans (98.7%), with
accepted RGB offsets up to 48 ms and P95 32 ms. RGB delivery was still about
21 Hz. A 15-minute 50 ms run paired 8,820 of 9,034 scans (97.6%); 211 failed
the RGB limit and two were lost from the adapter queue. It produced 5,792
finite odometry messages and 5,792 nonempty registered clouds, with a 1.065 s
maximum odometry receive gap. RSS peaked at 744,688 KiB, with 3,220 visual
points and 3,911 LiDAR voxels near the end; this route covered less map area
than the 30 ms run, so the lower RSS cannot be attributed to the wider window.
The wider window is a low-speed simulation trial, not a flight-ready timing
setting. The default remains 30 ms. An ASAN/UBSAN build with a temporary
100-point budget exercised over 900 evictions and reported no sanitizer
errors; core, adapter and clock bridge exited at code 0. LeakSanitizer was
disabled, and instrumented throughput is not representative.

Headless Gazebo (`gz_sensors.launch.py world:=astra_motion.sdf
headless:=true rviz:=false`) renders the same LiDAR and RGB topics without a
GUI. In a five-minute 30 ms moving run with the original reliable bridge
publisher and best-effort adapter image subscriber, 2,934 of 3,212 scans
paired (91.3%); 273 failed the RGB limit and four lacked an IMU window. It
produced 2,259 finite odometry messages and 2,259 nonempty clouds, with a
0.925 s maximum odometry receive gap. A second five-minute run changed only
the RGB bridge publisher to sensor-data best-effort QoS: 3,187 of 3,435 scans
paired (92.8%), with 247 RGB rejections. These runs have different map and
simulated-time coverage; this small change is not evidence that best-effort
transport solves the missing RGB frames. A direct Gazebo image-topic frequency
sample was near 30 Hz, while the ROS image topic was about 20-23 Hz under the
same workload. The bridge QoS was restored to its original setting after the
comparison.

With the reliable bridge publisher and a matching reliable adapter image
subscription, a ten-minute headless 30 ms moving run delivered RGB at about
30.3 Hz and paired 6,127 of 6,130 scans (99.95%). No scans failed the RGB
limit; two lacked an IMU window and there was no adapter queue overflow.
Accepted RGB offsets had mean 7.76 ms, P95 16 ms and maximum 16 ms. The probe
recorded 4,790 finite odometry messages and 4,789 nonempty registered clouds;
the maximum odometry receive gap was 0.728 s. Peak core RSS was 923,956 KiB.
Visual pruning was active, but retained-image memory and RSS were still near
their highs at the end of ten minutes. This is a simulation-only synchronization
improvement; larger-route memory and time-aligned moving accuracy remain open.

The first commanded 4 m square run failed before three minutes despite healthy RGB
pairing (1,904 of 1,905 scans, no RGB-limit rejection). At about 187 s of
Gazebo simulation, the body was near (1.25, 0.98, 1.0) m while odometry was
near (2,792, -46,488, 402) m in its own startup frame. The core reported
zero effective LiDAR features and `NaN` residual summaries, then continued
mapping. LiDAR voxels grew past 1.16 million; RSS reached 1,684,092 KiB and
only 1,190 of 1,316 odometry updates had a nonempty registered cloud. A
guarded reproduction stopped motion at 80.6 s wall time when an adjacent
odometry step reached 0.933 m. At that point the body was near (4.05, 0.98) m
and odometry near (4.03, -0.43) m. This confirms a repeatable loss of moving
localization on the wider route, not a LiDAR/RGB pairing failure. The large
RSS increase is dominated by the unbounded LiDAR voxel map after divergence;
the smaller-loop runs were dominated by retained grayscale images. A finite
pose alone is therefore not a sufficient stability check.

The earlier `feats_down_body` shutdown crash was an allocator ABI mismatch:
system PCL allocated with 16-byte Eigen alignment while this package freed
with a different Eigen path. The core now uses 16-byte alignment and the
matching malloc/free path. `PointToPlane` initialization and a leaked VIO
patch allocation were also corrected. The former sensor `bridge_node` SIGINT
fault was avoided by using `parameter_bridge`. The final proximity-filter change passed
a further ASAN/UBSAN static run with no reported sanitizer errors and clean
SIGINT exits for core, adapter and clock bridge. LeakSanitizer was disabled
for that run; RSS behavior was measured separately. Instrumentation reduced the
core rate to about 1 Hz and caused LiDAR queue drops, so it is not a rate test.

**Status: strict 30 ms RGB pairing is stable in the tested ten-minute headless
simulation with matched reliable QoS, but moving localization fails on the
obstacle-approach route and broader-route memory is unbounded after divergence. Outputs remain
for debugging only and must not be connected to a planner or flight
controller.** Diagnose the LiDAR correspondence loss, enforce estimator output
validity and map-memory limits, and pass time-aligned moving accuracy plus
longer varied-route continuity tests before adding a 3D local map or
EGO-Planner. RViz2 is optional and is not an acceptance condition.

### 2026-10-06 correspondence investigation

Live Gazebo PointCloud2 has no per-point timestamp. The adapter's 0.1 s
organized-index time is synthetic, though its units are correct for the
Velodyne-style preprocessing path (`time` in microseconds; optional Livox
`offset_time` in nanoseconds). A 1 ms synthetic period on the wide route still
failed at 67.9 s with a 0.701 m odometry step and complete image pairing.
Setting all point times to zero is not an equivalent deskew-off test because
the preprocessor then generates yaw-based time offsets. Raw rings are 0..15,
but `scan_line` and `ring` do not affect the current non-feature Velodyne
handler when time is present. The regular 16-line Gazebo cloud must not use the
real Mid-360's four-line/CustomMsg configuration.

The earlier simulated lidar sensor was 0.04 m above `mid360_link`, although its
cloud was labeled with that link. Ground returns near z=-1.22 m confirmed the
scan origin. Historical trials used LiDAR-to-IMU Z **-0.10 m**, whose sign was
later found inconsistent with `p_imu=R_IL*p_lidar+t_IL`; the required value is
**+0.10 m**, while LiDAR-to-camera optical Y is -0.16 m. Those earlier wide-route trials failed
at 80.5 s (0.670 m step) and 80.8 s (0.687 m step), respectively. The configured
rotation is unchanged. The previous claim that all transform directions matched
was incorrect. The 1.17 runtime now labels points `mid360_scan_frame`, checks
expanded SDF/TF/calibration before flight, and uses the corrected translations.
Its explicit runtime profile publishes `base_link` position with the rotated
IMU-to-body offset [0,0,-0.12]; internal filtering remains at the IMU. Historical
flight results still refer to their original models and configurations.

Disabling the visual update did not cure the failure: LIO-only with the
original 0.5 m voxel size reached a 2.298 m step at 80.6 s. Its effective
correspondences had declined from about 4,070 at rest to below 2,000 before
the turn, then fell to zero; the residual log became `-nan` while the global
LiDAR map continued to grow. Raising only `lio.voxel_size` to 1.0 m kept
LIO-only steps below 0.046 m for 130 s with at least 1,846 correspondences,
but odometry stayed near y=0 during the commanded lateral leg. Actual Gazebo
y was not recorded, so that run is not an accuracy pass. Full fusion with the
1.0 m voxel still failed at 80.6 s
(0.678 m step). The visual plane-map lookup is hard-coded to 0.5 m, so a
non-default LiDAR voxel size is not a clean full-fusion configuration.

At half speed with doubled segment durations, preserving the 4 m route,
the original full-fusion configuration failed on the same lateral leg at
147.0 s (0.579 m step). Pairing was 1,778/1,779 with zero RGB rejections;
afterward the core logged zero correspondences and `-nan` residuals and grew
past 416,000 LiDAR voxels before shutdown. The small-loop original-parameter
control completed 90 s with 1,285/1,288 pairs, 0.056 m maximum step, finite
odometry, and no zero-correspondence log entries. All core/adapter processes
exited cleanly on SIGINT; the guarded probes deliberately exited nonzero on
a threshold breach. The Gazebo wrapper reports -2 when interrupted.

The earlier claim that low horizontal plane diversity caused the lateral
failure was premature. The 131 s `min_eigen_value=0.0025` run had bounded
steps and odometry y near zero, but actual Gazebo y was not recorded. A
time-aligned repeat with the old one-shot `gz topic` driver showed that a
commanded lateral leg could leave the model at y=0; this does not test lateral
localization. The endurance driver now holds a Gazebo Transport publisher and
repeats commands every 0.2 s.

`audit_trajectory.py` records the Gazebo model pose and FAST-LIVO2 odometry
independently, pairing them by simulation timestamp within 50 ms. With
repeated commands, the LIO-only 1.0 m voxel run tracked Gazebo through y=0.969 m
with 0.076 m planar error, then jumped at y=0.975 m to odometry y=-0.569 m
(1.544 m planar error). Full fusion with `min_eigen_value=0.0025` already had
about 0.55 m forward error before the turn and reached 1.014 m planar error
near the same boundary. The test model collides with a box at x about 4 m,
limiting actual lateral motion to about 0.975 m, so the nominal route never
tests the intended 4 m lateral displacement. The separate Gazebo-only y
command confirms the velocity plugin accepts lateral motion.

Raw-scan plane normals include many y-facing surfaces near the obstacle; weak
lateral geometry remains a hypothesis, not an established root cause. Earlier
zero correspondences, NaN residuals, and global-map growth remain confirmed
post-divergence hazards. Next, use a collision-free route with continuous
time-aligned truth recording and log accepted correspondence normals and
filter conditioning at the onset of drift. No wide-route localization or
LiDAR-memory repair is validated. Keep EGO-Planner and PX4 disconnected.

The first collision-free short-route audit used the original full-fusion
parameters and repeated commands: 0.5 m forward, 0.5 m lateral, then 0.5 m
reverse, with four-second stops. Gazebo stayed within x=-0.001..0.499 m and
y=0..0.498 m, well clear of the boxes. All 522 odometry poses paired with
Gazebo within 16 ms; maximum planar error was 0.086 m during the lateral leg
and final error was 0.010 m. Maximum odometry step was 0.034 m, adapter
pairing was 819/820, and no zero correspondence or NaN residual occurred.
Effective correspondences stayed above 2,646 during motion. Core RSS rose
from 182,752 to 303,448 KiB over 65 s, so memory stability remains open.
The time-aligned traces and per-segment metrics are in the investigation
report's output directory. Probe, core, and adapter exited 0; Gazebo wrapper
reported -2 on SIGINT. This validates only the short clear route, not the
wide route or obstacle approach. EGO-Planner and PX4 remain disconnected.

A five-loop extension of that clear route ran for 310 s with original full
fusion. Gazebo remained within x=-0.004..0.499 m and y=-0.002..0.501 m, and
all 2,434 odometry poses matched Gazebo within 20 ms. Pairing was
3,199/3,201 scans; no non-finite odometry, zero correspondence, or NaN
residual appeared. Maximum planar error was 0.159 m, with recurring brief
return-lateral errors around 0.15 m in loops 3-5; maximum adjacent odometry
step was 0.144 m. A single loop-3 LIO iteration dropped to 111 effective
correspondences, then recovered. Final planar error was 0.0013 m. RSS rose
from 179,564 to 470,356 KiB; the final 20 s stationary tail was flat, but
RSS still increased 17,264 KiB over the last minute. Five-minute moving
accuracy and long-term memory stability therefore need a defined acceptance
threshold and further investigation before increasing route size. EGO-Planner
and PX4 remain disconnected.

The same route completed ten loops over 590 s with 4,650 time-aligned poses,
6,048/6,049 sensor pairs, no NaN or zero correspondences, and 0.0062 m final
planar error. Transient planar error reached 0.2499 m and the largest adjacent
step 0.2562 m, so endpoint accuracy still masks moving errors. RSS rose from
183,072 to 520,868 KiB; the final minute added 3,844 KiB. Retained grayscale
image data rose by 287.4 MiB, while LiDAR voxels added only 1,395 and were
flat at 2,543 during the last minute. The visual-point cap did not prune:
only 2,205 points were retained, but their observations kept 986 distinct
full images (288.9 MiB). Image retention is the dominant measured RSS source;
a sustained moving-memory bound remains unverified. The earlier 111-
correspondence scan had 461 raw and 246 downsampled points, a finite 0.0434
residual and 3.302 ms LIO total time; nearby map/RSS counters showed no
explosion. The ten-loop run had no count below 500. Larger routes, local-map
integration, EGO-Planner and PX4 remain on hold.

The Gazebo profile now sets `vio.max_retained_images=512` (generic core default:
unlimited). Before each visual update, the core removes observations from the
oldest distinct image frames; points with no remaining observations are also
removed. The combined launch accepts `max_retained_images:=N`. Five-second
status logs report retained image count and MiB plus cumulative pruned image,
observation, and point counts. On the same ten-loop route over 590 s, 4,724
odom poses paired with Gazebo within 20 ms. Final error was 0.0071 m, maximum
transient error 0.1917 m, and maximum step 0.1704 m. RSS rose from 186,856 to
353,176 KiB, adding 1,532 KiB in the final minute; the uncapped run ended at
520,868 KiB and added 3,844 KiB in its final minute. Retained image data
settled near 150 MiB with 512-513 images in five-second snapshots (one new
frame can be added after the pre-update pruning pass). At shutdown the map
held 512 images, 1,210 visual points, and 2,534 LiDAR voxels. The image cap
addresses the measured short-loop memory source, but this ten-minute run does
not prove an indefinite RSS bound or fix the return-leg pose transient. The
larger route and planner/PX4 integrations remain on hold.

The 512-image setup then completed 21 loops over 20 minutes. RSS peaked at
367,416 KiB and ended at 365,672 KiB; during the final ten minutes it stayed
within 365,136-367,416 KiB. Retained image data stayed near 150 MiB. This is
a demonstrated plateau on the repeated 0.5 m route, not a bound for wider
maps. All 9,609 odometry poses paired with Gazebo within 20 ms, but maximum
moving planar error was 0.2608 m and the largest adjacent step 0.2003 m.
Each of the 21 loops exceeded 0.10 m maximum error; 488 poses exceeded
0.10 m and 55 exceeded 0.20 m. Final error was 0.0032 m. The model half-width
is 0.175 m, so an adverse 0.261 m pose error can consume 0.436 m of planned
center-to-obstacle clearance before other uncertainty. No planner was run.

The audit driver supports `--speed-scale 0.5`, doubling movement duration so
each leg still travels 0.5 m at 0.025 m/s. A five-loop half-speed run had
0.1231 m maximum error and eight poses above 0.10 m, versus 0.2485 m and
125 such poses in the first five 0.05 m/s loops of the 20-minute run. Its
maximum odometry step remained 0.1510 m. This suggests speed sensitivity,
but one comparison does not establish cause or make the odometry safe for
near-obstacle planning.

The core stamps published odometry with the current ROS time rather than the
LiDAR update time. The <=20 ms Gazebo/odom pairing therefore checks the
published timestamps, not estimator processing latency. Future audits should
log both times and the pre/post LIO and VIO pose increments. Synthetic
point timing and matching conditioning remain open hypotheses. EGO-Planner
and PX4 stay disconnected.

An optional timing audit is available from the combined launch with
`timing_audit_csv:=/absolute/path.csv`. It logs each LIO/VIO cut time, the
ROS times at sync readiness, after IMU propagation and after update, the
LIO odom publish time, and the state position/yaw before and after each
stage. The audit leaves odom timestamps and estimator calculations unchanged;
VIO updates are internal and do not publish odom directly. In a guarded
five-loop run, 2,460 odom poses paired with Gazebo. LIO measurement-to-publish
delay was 120 ms median, 136 ms P95, and 176 ms maximum. At the largest
0.1588 m odom step, the preceding VIO correction was 0.1850 m, followed by
0.0527 m IMU propagation and 0.0790 m LIO correction. Published-time error
was 0.1612 m; comparing the same estimate to measurement-time truth gave
0.1672 m. The timing difference therefore explains only millimeters of this
low-speed excursion. Visual update residuals and conditioning need further
inspection before changing estimator thresholds. EGO-Planner and PX4 remain
disconnected.

### CPU and scheduling audit

The optional timing CSV also records stage wall time, main processing thread
CPU time, whole-process CPU time, voluntary/involuntary context switches, and
monotonic start/end times. On Linux it also samples `/proc/thread-self/schedstat`
for main-thread runnable waiting time; missing counters are reported as -1.
Systems without enabled scheduler accounting may report zero waiting time,
which by itself does not prove absence of preemption.
Process CPU includes OpenMP workers and concurrent
ROS callbacks; it can exceed wall time. Wall minus main-thread CPU includes
preemption, synchronization and I/O waiting, so it is not a direct measurement
of scheduler wait or visual computation alone.

The Gazebo combined launch defaults to `openmp_wait_policy:=PASSIVE` and
`openmp_spin_count:=0`. These set `OMP_WAIT_POLICY` and `GOMP_SPINCOUNT` only
for the estimator process, allowing idle OpenMP workers to sleep rather than
spin while Gazebo and ROS need CPU. Estimator iterations, thread count, input
timestamps and freshness gates are unchanged. Both settings are configurable;
the generic core and other launch files keep their existing runtime defaults.
The zero spin count is specific to GCC's libgomp runtime. Recheck throughput
and latency on the target hardware before carrying this simulation setting
into a hardware deployment.

### Gazebo input reception

The combined Gazebo launch requests reliable raw LiDAR and IMU subscriptions
with bounded history depths of 10 and 100, respectively. The Gazebo bridge
must offer reliable publishers. Toggle `reliable_lidar_subscription:=false`
or `reliable_imu_subscription:=false` for best-effort sensor QoS; standalone
adapter defaults remain best effort for compatibility with hardware drivers.
Reliable delivery does not guarantee low latency. Original measurement stamps
and all input coverage/freshness checks remain in force.

With `timing_audit_csv` enabled, the core also writes `<path>.inputs.csv`,
recording point cloud, image and IMU callback entry and buffer insertion times.
This distinguishes publish-to-callback delay from callback work and later
estimator waiting. The file has a single writer on the receive executor and
uses buffered output; the normal runtime does not create it. Point cloud
callback tracing currently covers the standard PointCloud2 input used by
Gazebo, not the legacy Livox callback.

An opt-in Fast DDS profile is installed at `config/fastdds_sitl.xml`. It uses
UDP discovery and a 16 MiB shared-memory segment per participant. Source the
same ROS overlays and set `FASTRTPS_DEFAULT_PROFILES_FILE` to the absolute
profile path in **every** participating SITL terminal before launching the
sensor bridge, adapter, estimator, map and observers. Setting it after nodes
have started does not change their transport. This is a Fast DDS simulation
configuration; other RMW implementations do not use it. It increases shared
memory allocation and is not a hardware configuration. Unset the variable
and restart the participating processes to restore runtime defaults.

## Guarded local mapping prototype

The estimator now records visual-update quality in the optional timing CSV:
retrieved and accepted patches, mean squared photometric residual, pose
Hessian condition ratio, proposed planar correction, and whether the update
was rejected. `max_planar_correction_m` and `min_accepted_patches` are optional
launch arguments; both default to zero (disabled). When enabled, a rejected
VIO frame restores its pre-visual state and covariance before updating the
visual map. These thresholds are experimental and require a time-aligned route
comparison before use as estimator defaults.

For low-speed simulation mapping, start `ros2 launch astra_fast_livo2
local_mapping.launch.py` alongside the sensor and estimator launches. It
voxelizes `/fast_livo2/cloud_registered_full` in `camera_init`, retains a bounded 5 m local
window for 3 s, and publishes `/local_mapping/occupied` and
`/local_mapping/obstacles` at 2 Hz. Inflation defaults to 0.65 m as a provisional
vehicle-radius plus localization-error allowance. It detects non-finite,
regressing, or >0.12 m adjacent odometry updates, clears the map on a jump,
and requires ten consecutive good odometry updates plus a fresh cloud and at
least 20 occupied voxels before `/local_mapping/ready` becomes true.
`/local_mapping/diagnostics` reports the reason when it is false. Invalid
odom, mismatched frames, and bad cloud timestamps revoke readiness immediately;
the 50 ms health check revokes it when odom or cloud ages out.

The full registered cloud is published once per LIO update with the LiDAR
measurement timestamp, matching odometry. The older `/cloud_registered`
visualization topic is cropped to the RGB field of view in LIVO mode and can
contain empty messages; it must not be the sole input for obstacle mapping.
The `cloud_topic` parameter permits an explicit alternative source.
`self_filter_radius_m` defaults to zero (disabled). The isolated x500 obstacle
test uses 0.45 m to reject simulated rotor/airframe returns from new scans;
it does not erase already mapped environmental obstacles. A different
airframe requires a separately measured self-return envelope.

This map is an occupied-point prototype, not a raycast free-space occupancy
grid. Its readiness flag catches specific data faults, not every localization
error or every obstacle. PX4 output is disabled by default; the separate,
explicitly enabled SITL execution harness provides bounded Offboard tests.

In a 70 s short-route baseline with the visual cap disabled, 523 odom poses
paired with Gazebo within 16 ms. Gazebo covered approximately 0.5 m forward
and 0.5 m lateral without collision. Peak planar error was 0.1024 m, maximum
odom step 0.0552 m, and final error 0.0170 m. A map probe saw 129 occupied
and 129 inflated cloud publications, with readiness true 130 times and false
once for stale odometry. On a second 70 s run with only
`max_planar_correction_m:=0.04` changed, one VIO frame was rejected; 586 poses
paired within 20 ms, peak error was 0.0765 m, maximum step 0.0271 m, and
final error 0.0068 m. The two runs had different VIO correction distributions,
so this is a functional check of the cap, not evidence that 0.04 m is an
optimal threshold. The cap remains disabled by default.

## Planner-only integration

The vendored ROS 2 Jazzy EGO-Planner packages under `src/ego_planner_jazzy`
are documented in `UPSTREAM.md`. Build them in a separate overlay, then build
this launch package against that overlay:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/cyf/dev/astra_ros2_ws
source install/setup.bash
colcon --log-base log_ego build --packages-up-to ego_planner \
  --build-base build_ego --install-base install_ego --symlink-install \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
source install_ego/setup.bash
colcon --log-base log_ego_astra build --packages-select astra_fast_livo2 \
  --build-base build_ego_astra --install-base install_ego_astra --symlink-install
```

With Gazebo, FAST-LIVO2, and `local_mapping.launch.py` running, source
`install_ego/setup.bash` and `install_ego_astra/setup.bash`, then run:

```bash
ros2 launch astra_fast_livo2 ego_planner_guarded.launch.py
```

This starts only `ego_planner_node`. It reads `/fast_livo2/odom`, the
world-frame `/local_mapping/obstacles` cloud, and transient-local
`/local_mapping/ready`; it accepts `geometry_msgs/msg/PoseStamped` goals on
`/move_base_simple/goal` in `camera_init` and publishes
`traj_utils/msg/Bspline` on `/planning/bspline`. It rejects goals until
readiness is true, clears its goal when readiness becomes false, and requires
a new goal after recovery. The default planning speed and acceleration are
both 0.5 m/s and 0.5 m/s^2. The map is already inflated, so the planner's
second inflation is set to zero. The planner's `traj_server` is not started;
no trajectory is executed or bridged to PX4.

`/planning/trajectory_valid` is a transient-local Boolean for downstream
consumers. It becomes true with a new B-spline and false on map withdrawal,
planning failure, trajectory completion, or odom-to-planned-position error
above `max_tracking_error_m` (default 0.30 m). The planner also expires the
mapping readiness heartbeat after
1.25 s without messages, even if the last latched readiness value was true.
A future executor must require fresh readiness and validity, check trajectory
age and identity, and stop on either false signal. The validity topic alone
does not command a hover or invalidate an already received B-spline in an
unconnected consumer.

An isolated ROS gate test produced zero B-splines before readiness, two after
a 0.6 m goal while ready, and none after readiness was revoked. In the
`astra_motion.sdf` stationary Gazebo scene, the guard published 2,802
inflated points in `camera_init`, and a 0.5 m goal produced one B-spline with
11 control points. Stopping the estimator made readiness false and a later
goal produced no B-spline. In a separate 20 s manual forward run at 0.025 m/s,
Gazebo moved 0.496 m and odom moved about 0.496 m; readiness stayed true and
the planner produced three B-splines for a goal sent during motion. This
verifies the interface and interlock, not time-aligned moving-route accuracy,
trajectory clearance, or flight safety.

## Stationary obstacle and trajectory audit

The dedicated `astra_planner_obstacle.sdf` world places a 0.6 m box at
`(1.8, 0, 1)` on the forward route. Launch it through `gz_sensors.launch.py`
with `world:=astra_planner_obstacle.sdf`, then run the estimator, mapping guard,
and planner with `planning_horizon:=3.5`. The longer horizon places a 3.2 m
goal beyond the inflated box; the 2.0 m default placed the local target inside
the obstacle and produced no trajectory.

```bash
ros2 launch astra_fast_livo2 ego_planner_guarded.launch.py planning_horizon:=3.5
ros2 run astra_fast_livo2 audit_planner_trajectory.py --goal-dx 3.2 --seconds 7
```

The audit samples each cubic B-spline every 10 ms, computes its analytic
velocity and acceleration, compares samples with a recent inflated obstacle
cloud, and checks gaps between consecutive trajectory revisions. Its defaults
are 0.15 m additional cloud clearance, 0.5 m/s speed, 0.5 m/s^2 acceleration,
0.05 m position revision gap, and 0.1 m/s velocity revision gap. A failed
check returns nonzero. The tool depends on `python3-scipy`.

An initial 0.5 m goal exposed 1.322 m/s^2 acceleration despite a 0.5 m/s^2
configuration. The planner now checks derivative control-point vector norms
after refinement and stretches knot time before publication. In the final
stationary box test, the first 3.2 m trajectory had 0.381 m minimum distance
to the already inflated cloud, 0.493 m/s peak speed, and 0.335 m/s^2 peak
acceleration. A second revision had a 0.014 m position gap. With no executor
attached, the stationary model eventually lagged the planned position by
0.30 m; the planner then revoked validity. A 2.5 m goal inside the inflated
box produced no B-spline and left validity false. Planning failures are
limited to three attempts per goal.

Injecting a 2 m odom step made readiness false in 2.6 ms and trajectory
validity false in 2.8 ms; a goal sent while false produced no new B-spline.
Stopping the mapping process while a trajectory was active revoked validity
in 1.02 s even though the last readiness message remained true. These are
single-run simulation measurements. The cloud is occupied points, not a
free-space proof; the vehicle did not execute the obstacle route, and moving
localization error was not measured in this trial. PX4 remains disconnected.

## Gazebo snapshot timing (2026-10-08)

The combined `fast_livo2.launch.py` now defaults to `point_time_mode:=snapshot`.
Gazebo Sensors 8 `GpuLidarSensor::Update` renders one frame at `_now` and stamps
the entire point cloud at that time; it does not provide individual firing
times. Snapshot mode preserves this stamp and assigns zero point offsets.
The VELO16 preprocessor explicitly accepts those zeros instead of inferring
a rotating scan from point yaw. This mode is specific to the simulated sensor.

The adapter selects the first image at or after the cloud measurement, within
`max_lidar_image_delta_sec` (30 ms by default), and requires IMU coverage through
that image timestamp. The core permits bounded snapshot-to-image propagation
without waiting for another LiDAR frame. Points keep their cloud acquisition
time during IMU compensation; the LIO state/odom timestamp is the actual image
fusion time. Missing coverage or out-of-window images are rejected.

For a timing comparison or rollback, use `point_time_mode:=synthetic_scan`.
The standalone adapter and generic C++ core retain their previous defaults.
Physical Mid-360 data must use its actual acquisition times and calibrated
extrinsics. No freshness threshold is relaxed, and IMU rate odometry remains
disabled pending an audit of its shared state and frame ID.

### Input event audit and pending VIO scheduling

Set `input_audit_csv:=/absolute/path.csv` to record adapter callback entry,
buffer insertion, synchronization checks and cloud/image publication times.
Each event has both a monotonic wall time and the local ROS simulation clock;
the optional logger batches writes in a background thread. Keep the source
measurement timestamps distinct from callback arrival times. A late observer
callback is not evidence that the estimator itself published late.

The core no longer requires a new cloud to finish the pending visual update
for an already completed LIO cut. The image and IMU buffer guards remain in
place, and the following LIO cut still requires new LiDAR data. The previous
generic cloud guard could wait for the next 10 Hz scan before running VIO.
The change preserves sequential updates to the same state and measurement
time; it does not introduce independent LIO/VIO threads. The `test_pending_vio`
regression exercises the real synchronizer with an empty cloud queue and
checks that the other input guards remain effective.
