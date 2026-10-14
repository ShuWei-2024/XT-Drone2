# EGO-Planner ROS 2 Jazzy core

Source: `JIMT111/ego-planner-swarm-jazzy-fixed`, branch `ros2_version`,
commit `c18531594ed639ee605ed76fb2bb87310b8dcb88`.

This directory contains only the six packages needed to build the planner
and its messages: `plan_env`, `path_searching`, `bspline_opt`, `traj_utils`,
`quadrotor_msgs`, and `ego_planner` (in `plan_manage`). The upstream GPL-3.0
license is preserved in `LICENSE`.

Local changes in `plan_manage`:

- Require `/local_mapping/ready` before accepting odometry or a manual goal
  and before running the planning or collision timers. A false value clears
  the active target and requires a new goal after recovery.
- Reject goals with a different frame ID from odometry and use the goal's
  requested height rather than a hard-coded 1 m height.
- Handle goals arriving during FSM initialization without blocking the
  executor while waiting for an already-running trajectory.
- Publish `/planning/trajectory_valid` and revoke it on map withdrawal,
  completed or failed plans, or excessive odom-to-trajectory tracking error.
- Recheck vector speed and acceleration bounds after refinement, scaling the
  spline time before publication when necessary.
- Use the node's ROS clock for trajectory timing and stop after three failed
  planning attempts per goal.
- Expire the mapping readiness heartbeat using a steady clock, including
  unexpected mapping-process exit. The Astra launch uses a 0.30 s timeout;
  the source default and launch/runtime overrides must be distinguished.
- Declare the direct `std_msgs` dependency used by the readiness input.

The upstream simulation launch and trajectory server are present for source
fidelity but are not started by the Astra planner-only launch. No PX4 code or
command bridge is included in this integration.

Further local changes for guarded outdoor execution:

- Consume a frozen `/planning/execution_envelope` shared with the flight
  supervisor and preview; reject changed or invalid envelopes.
- Use a bounded global envelope guide across the local search window, with
  sensor-grid fallback and progress-aware local targets.
- Share planning budgets across search/optimization attempts and record
  callback timing; avoid masking blocked planning with a separate heartbeat.
- Check complete spline segments, preserve geometry during derivative-bound
  retiming, and validate fallback boundary states and stationary endpoints.
- Publish trajectory-valid acceptance after the curve and handoff checks,
  preserving continuity across replans.

These changes do not certify every route. Current envelope-search rejection
diagnostics remain incomplete, and same-configuration repeated outdoor flight
has failed. See the [architecture](../../../docs/ARCHITECTURE.md) and
[validation index](../../validation/README.md) for current scope and evidence.
