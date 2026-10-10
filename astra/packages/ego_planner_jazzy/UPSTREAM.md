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
- Expire the mapping readiness heartbeat after 1.25 s of silence using a
  steady clock, including when the mapping process exits unexpectedly.
- Declare the direct `std_msgs` dependency used by the readiness input.

The upstream simulation launch and trajectory server are present for source
fidelity but are not started by the Astra planner-only launch. No PX4 code or
command bridge is included in this integration.
