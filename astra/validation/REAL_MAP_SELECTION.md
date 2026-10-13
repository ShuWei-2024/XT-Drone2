# First real-scan map candidate: Replica office0

Selected on 2026-10-11 for the isolated PX4 1.17 / Gazebo Harmonic pipeline.

## Source

- Original dataset: https://github.com/facebookresearch/Replica-Dataset
- Paper: https://arxiv.org/abs/1906.05797
- Selected downloadable derivative: https://huggingface.co/datasets/voviktyl/Replica-SLAM
- Pinned revision: `7d7ae90b48035aa2e9baa58018f5e090bd6d5dc5`
- File: `cull_replica_mesh/office0.ply`
- Size: 22,569,023 bytes
- SHA256 advertised by the mirror's LFS metadata:
  `0dd4b4d292cc325b52eeb596b0d089440cc333df8757cced7fa12daedb9309a6`
- Local cache: `astra/runtime/map_candidates/replica_office0/` (ignored by Git).

Replica contains reconstructions of real indoor spaces. The selected mesh is a
**culled derivative**, not the complete original scene. Its header describes
589,517 vertices, 1,010,496 triangular faces and RGBA vertex colors. Original
Replica HDR/Ptex textures are not included in this file. The official full
compressed dataset is approximately 33.86 GB, so the single-scene derivative
is a practical first geometry inspection asset.

The original Replica Research Terms permit non-commercial or not-for-profit
research and educational use. Preserve attribution and the upstream terms;
keep downloaded and converted assets out of the project Git repository.
Dataset use is separate from the project code license.

Download completed and SHA256 matched on 2026-10-11. Binary mesh inspection
confirmed finite coordinates and valid triangle indices. Raw axis spans are
approximately 4.40 x 5.01 x 2.99; metric scale and gravity alignment still require
verification. See `real_map_office0_inspection.json` for the measured bounds.

## Integration status and required checks

This selection does not certify the map for flight and does not change the
default world, sensor settings, flight bridge or PX4 parameters.

1. Verify downloaded bytes against the SHA256 and inspect scene bounds.
2. Establish floor, gravity axis, metric scale and a transform into Gazebo ENU.
3. Inspect culling holes and ceiling clearance; identify a clear spawn area.
4. Convert to a Gazebo-supported visual mesh and preserve vertex colors through
   a supported material/texture path. LiDAR needs rendered surfaces; RGB also
   needs useful visual contrast. Merely displaying a point cloud is insufficient.
5. Validate collision surfaces independently. Do not treat the culled mesh as a
   closed solid or use its missing surfaces as proof that space is free.
6. First perform a passive sensor/render check. Then define a short, low-speed
   route with generous clearance before enabling a closed-loop SITL test.

The intended data flow remains imported scene -> simulated LiDAR/RGB/IMU ->
FAST-LIVO2 -> guarded local map -> EGO-Planner -> PX4 SITL. The prebuilt mesh
must not masquerade as live registered sensor data or enter flight setpoints
as Gazebo truth. External Vision remains disabled.

Existing limitations remain: occasional roughly 0.12 m height transients and
unproven wide-map / indefinite memory stability. This asset is for simulation,
not hardware flight certification.

## Gazebo integration

`astra/tools/prepare_replica_office0.py` converts the pinned source into an
independent scene under `astra/runtime/maps/replica_office0`. It preserves the
native Z-up geometry, translates the sampled spawn region to XY zero, and
places the observed floor just below an invisible support plane at Z zero.
The support plane is synthetic; room geometry and baked colors come from the
scan. The source's metric scale is retained without resizing.

The visual and static concave collision meshes each contain 100,000 triangles,
reduced from 1,010,496. RGB vertex colors are interpolated into a standard UV
atlas. COLLADA includes explicit surface normals for Ogre2 rendering. The
original HDR/Ptex textures are not used. Surface culling and mesh reduction
can leave or enlarge gaps. Passive integration and the four-leg short route
passed. Three subsequent repeats passed twice and stopped once on height
disagreement; room-scale scan traversal remains uncertified.

The largest sampled above-floor XY clearance is about 0.92 m at the spawn
location. That estimate uses visible scan surfaces; it is not a certificate
that missing surfaces or unknown regions are free.

Prepare the converter in an isolated venv (system NumPy/SciPy/Pillow are used):

```bash
cd /home/cyf/dev/XT-Drone2-astra
python3 -m venv --system-site-packages astra/runtime/map_tools_venv
astra/runtime/map_tools_venv/bin/pip install -r astra/config/map_tools_requirements.txt
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_replica_office0.py
```

The converter verifies the source SHA256 and refuses to overwrite an existing
output directory. Raw and converted dataset assets remain ignored by Git.

Run the imported scene with sensors, FAST-LIVO2, the guarded local map and
EGO-Planner, with no flight-command process:

```bash
cd /home/cyf/dev/XT-Drone2-astra
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
 -u PYTHONPATH -u LD_LIBRARY_PATH bash --noprofile --norc -c \
 'bash astra/run_117.bash static replica_view_01 --world-preset replica-office0 --seconds 600'
```

Use a new run name each time. While it is running, open the Gazebo GUI from
another clean terminal:

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

The original world is still the default. `--world-preset replica-office0`
supports static checks and explicitly prepared mesh-guarded obstacle-mode
tests. It rejects timer/small modes and unprepared routes. The previous large
outdoor rectangle does not fit this room.

`probe_scan_scene.py` records camera images, finite LiDAR returns, odometry,
registered clouds and readiness to distinguish loading a mesh from actually
observing it through the sensor pipeline. This passive check does not certify
motion accuracy or collision-safe closed-loop flight.

## Completed passive test (2026-10-11)

Run: `astra/runtime/replica_office0_static_117_03`, 60 seconds of passive
observation after startup. Result and passive health checks passed. Observed:

- LiDAR about 10 Hz, camera about 30 Hz, IMU about 100 Hz; odom about 10 Hz.
- 595 probe odom messages; 11,519 finite LiDAR returns per observed scan.
- Camera 640 x 480 with meaningful color/contrast; no longer an all-white image.
- Local-map readiness remained true during the observation window.
- Simulation real-time factor approximately 0.9995.
- Monitor odom measurement age at callback p95 28 ms, max 64 ms; this is not
  the odom age at a flight-control tick.
- No armed state or reset events observed. Original parameter/source baseline
  hashes preserved; all owned simulator processes exited.
- At that stage, 23 route/runner tests passed, including refusal to arm the
  scan without a prepared route and mesh guard.

The first two attempts loaded geometry and produced LiDAR/odom but had a white
camera image and zero visual map points. Explicit PBR texture binding and
reduced lighting alone did not resolve it. Adding explicit COLLADA surface
normals yielded a textured camera image and visual features in the third run.

See `replica_office0_integration_117.json` and `replica_office0_camera_117.png`.
This proves passive scan/sensor/estimator/map integration, **not indoor motion
or obstacle-flight accuracy**. This passive run used no trajectory execution.

## Guarded motion follow-up (2026-10-11)

A mesh distance guard, measurement-time sensor capture and flight auditor are
now available. The first five isolated flight attempts all terminated on the existing
0.20 m ground-frozen PX4/LIVO height-difference gate and landed/disarmed. Two
full-fusion runs executed part of the first backward leg (actual X spans about
0.405 and 0.529 m); no complete indoor route or blocked-path avoidance passed.

Raw LiDAR-to-surface residual p95 was about 4.6 mm. Raw IMU acceleration agrees
with measured motion, and the instrumented LIO integration covered its time
interval. Closing the camera did not remove the climb error. LIO matching
changes the vertical velocity estimate; the reason for its erroneous
corrections remains under investigation. A 0.25 m voxel diagnostic also failed;
the normal 0.5 m configuration remains the default.

One earlier recovery descended below the 0.55 m clearance margin after leaving
the inspected XY envelope. Execution-only clearance statistics had hidden
this. Audits now include fault hold and landing; the controller monitors mesh
clearance throughout recovery and has a 0.6 s measured-motion boundary forecast.
This forecast is an early abort reserve, not a stopping guarantee.

The current prepared route is smaller: 0.20 m backward, 0.10 m sideways, then
return. Its entire inspected recovery box is checked against known surfaces,
including descent. A grid covering-radius correction gives a conservative
continuous-box distance lower bound of **0.6148 m**, above the 0.55 m gate.
Only floor triangles wholly at/below Z=0.12 m are excluded for takeoff/landing.
Missing scan surfaces remain uncertified. This route passed once and then
passed two of three independent repeats. The other repeat safely stopped on
the 0.20 m height-difference gate. This is not room-scale scan certification.
The separate synthetic closed hall completed two approximately 30 m routes,
but later handoff auditing invalidated their full certification. The handoff
fix passed one full 29 m run; its independent repeat stopped on odom staleness. See
`REPLICA_FLIGHT_117.md` and `INDOOR_TRAVERSAL_117.md` for the distinct evidence.

Prepare and inspect the guard before any further isolated test:

```bash
python3 astra/tools/prepare_scan_flight.py
```

Reproducible diagnostic command, from a clean environment (new run name):

```bash
bash astra/run_117.bash obstacle replica_diagnostic_next \
 --world-preset replica-office0 --calibrate-ground-height \
 --route-json astra/config/route_replica_office0.json \
 --lio-only-diagnostic --capture-scan-inputs
# After all simulator processes exit:
python3 astra/tools/audit_scan_flight.py astra/runtime/replica_diagnostic_next
python3 astra/tools/audit_scan_inputs.py astra/runtime/replica_diagnostic_next
```

Keep the normal sensor noise and flight gates. External Vision remains off.
No hardware testing is covered. Evidence and commands:
`replica_office0_motion_117.json`, `REPLICA_FLIGHT_117.md`.
