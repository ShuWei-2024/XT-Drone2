"""Static SITL recovery graph, prepared disarmed; never an EGO obstacle map.

Every edge lies inside the same eroded flight envelope used by the planner.
Roots lie inside eroded descent prisms. Runtime recovery needs a valid frozen
PX4 transform and fresh PX4 state; stale LIVO is never used to move the target.
"""
from collections import deque
import numpy as np

from planning_envelope import PlanningEnvelope


class ConnectedRecovery:
    def __init__(self, envelope, descent_boxes):
        """descent_boxes must be expressed in the envelope's absolute XY frame."""
        self.envelope = envelope
        self.shape = envelope.free.shape
        # PlanningEnvelope's boxes are relative to the ground reference.
        bounds = np.array([[0., 0.], [self.shape[1], self.shape[0]]])*envelope.resolution
        # Cover 0.30 m estimator disagreement + 0.08 m arrival error + the
        # 0.25 m descent neighbourhood, in addition to grid corner padding.
        landing = PlanningEnvelope(np.asarray(descent_boxes)-envelope.origin,
                                   bounds, envelope.origin,
                                   resolution=envelope.resolution, reserve=max(.65,envelope.reserve))
        seeds = landing.free & envelope.free
        self.parent = np.full(self.shape, -1, dtype=np.int32)
        flat = self.parent.ravel()
        roots = np.flatnonzero(seeds)
        if not len(roots):
            raise ValueError('Connected recovery has no certified landing roots')
        flat[roots] = roots
        width = self.shape[1]
        free = envelope.free.ravel()
        queue = deque(map(int, roots))
        while queue:
            i = queue.popleft()
            y, x = divmod(i, width)
            for j in ((i-1 if x else -1), (i+1 if x+1 < width else -1),
                      (i-width if y else -1), (i+width if y+1 < self.shape[0] else -1)):
                if j >= 0 and free[j] and flat[j] < 0:
                    flat[j] = i
                    queue.append(j)
        if np.any(envelope.free & (self.parent < 0)):
            raise ValueError('Flight envelope contains cells without a recovery route')
        self.landing_cell_count = len(roots)

    def route(self, xy):
        """Return actual start and adjacent-cell centres, with collinear runs merged."""
        xy = np.asarray(xy, float)
        if not self.envelope.contains(xy):
            raise ValueError('Recovery start outside shared flight envelope')
        x, y = np.floor((xy-self.envelope.origin)/self.envelope.resolution).astype(int)
        width = self.shape[1]
        i = int(y*width+x)
        flat = self.parent.ravel()
        if flat[i] < 0:
            raise ValueError('Recovery start has no landing route')
        indices = [i]
        for _ in range(flat.size):
            j = int(flat[i])
            if j == i:
                break
            indices.append(j)
            i = j
        else:
            raise ValueError('Recovery graph cycle')
        # At a corner keep the turn cell. Axis-aligned runs never cut holes.
        keep = [indices[0]]
        for k in range(1, len(indices)-1):
            if indices[k]-indices[k-1] != indices[k+1]-indices[k]:
                keep.append(indices[k])
        if indices[-1] != keep[-1]:
            keep.append(indices[-1])
        centres = [self.envelope.origin +
                   (np.array([i % width, i // width])+.5)*self.envelope.resolution for i in keep]
        points = np.vstack([xy, centres])
        for a, b in zip(points[:-1], points[1:]):
            if not self.envelope.rectangle_free(np.minimum(a,b), np.maximum(a,b)):
                raise ValueError('Recovery edge leaves shared envelope')
        return points


def recovery_state_fault(node):
    """Independent recovery inputs. Does not waive PX4 validity or reset checks."""
    import time
    from px4_msgs.msg import VehicleStatus
    if not node.fresh() or time.monotonic()-node.local_wall > .25:
        return 'PX4/Gazebo recovery telemetry stale'
    p = node.local
    if not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid and
            p.heading_good_for_control):
        return 'PX4 recovery position/velocity/heading invalid'
    if not np.isfinite([p.x,p.y,p.z,p.vx,p.vy,p.vz,p.heading]).all():
        return 'Non-finite PX4 recovery state'
    if (p.xy_reset_counter,p.z_reset_counter,p.heading_reset_counter) != node.resets:
        return 'PX4 reset invalidated recovery transform'
    if node.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        return 'PX4 left Offboard during recovery'
    if not hasattr(node, 'recovery_transform') or not hasattr(node, 'recovery_z_translation'):
        return 'Frozen recovery transform unavailable'
    matrix, translation = node.recovery_transform
    xy = matrix.T @ (np.array([p.x,p.y])-translation)
    z = node.recovery_z_translation-p.z
    env = node.planning_envelope
    if not env.contains(xy) or env.z_bounds is None or not env.z_bounds[0] <= z <= env.z_bounds[1]:
        return 'PX4 recovery pose outside shared envelope'
    if node.truth_body is None or time.monotonic()-node.truth_wall > .25:
        return 'SITL recovery truth guard stale'
    if np.linalg.norm(xy-np.asarray(node.ground_route_origin_livo)-np.asarray(node.truth_body[:2])) > .30:
        return 'PX4 recovery XY disagreement exceeds 0.30 m'
    if abs(p.z+node.truth[2]-node.vertical_reference) > .35:
        return 'PX4 recovery Z disagreement exceeds 0.35 m'
    return node.mesh_fault() or node.mesh_guard.approaching_boundary(
        node.truth_body,node.truth_body_velocity,transit=True)


def execute_connected_recovery(node):
    """Follow prechecked static edges slowly to a landing root, then let PX4 land."""
    import time
    import rclpy
    from flight_recovery import landing_allowed
    fault = recovery_state_fault(node)
    if fault:
        node.event('connected_recovery_refused', reason=fault)
        return False
    matrix, translation = node.recovery_transform
    xy = matrix.T @ (np.array([node.local.x,node.local.y])-translation)
    try:
        points = node.connected_recovery.route(xy)
    except ValueError as exc:
        node.event('connected_recovery_refused', reason=str(exc))
        return False
    node.phase = 'connected_recovery'
    node.transit_active = True
    height, yaw = node.local.z, node.local.heading
    node.event('connected_recovery_started', route_livo_xy=points.tolist(),
               speed_limit_mps=.25, target_slew_acceleration_mps2=.25,
               source='prechecked static SITL graph; frozen PX4 transform; no stale LIVO')
    for a, b in zip(points[:-1], points[1:]):
        length = float(np.linalg.norm(b-a))
        deadline = time.monotonic()+max(12., length/.15+8.)
        command_xy = np.array(a)
        speed = 0.
        previous = time.monotonic()
        stable_since = None
        next_send = 0.
        while rclpy.ok() and time.monotonic() < deadline:
            node.receive_callbacks(timeout_sec=.005)
            fault = recovery_state_fault(node)
            if fault:
                node.event('connected_recovery_failed', reason=fault)
                return False
            now = time.monotonic()
            if now < next_send:
                continue
            dt = min(.075, max(0., now-previous))
            previous, next_send = now, now+.05
            actual = matrix.T @ (np.array([node.local.x,node.local.y])-translation)
            if np.linalg.norm(actual-command_xy) > .30 or abs(node.local.z-height) > .15:
                node.event('connected_recovery_failed', reason='Recovery tracking error exceeded')
                return False
            delta = b-command_xy
            remaining = float(np.linalg.norm(delta))
            desired = min(.25, np.sqrt(2*.25*remaining))
            speed = min(desired, speed+.25*dt)
            if remaining > 1e-9:
                command_xy += delta/remaining*min(remaining, speed*dt)
            ned = matrix @ command_xy+translation
            node.send((float(ned[0]),float(ned[1]),height,yaw))
            node.sample((float(ned[0]),float(ned[1]),height,yaw), reason='certified recovery')
            arrived = (remaining < .005 and np.linalg.norm(actual-b) < .08 and
                       np.hypot(node.local.vx,node.local.vy) < .08)
            stable_since = (stable_since or now) if arrived else None
            if stable_since is not None and now-stable_since >= .5:
                break
        else:
            node.event('connected_recovery_failed', reason='Recovery edge arrival timeout')
            return False
    if not landing_allowed(node.mesh_guard,node.truth_body[:2]):
        node.event('connected_recovery_failed', reason='Actual recovery endpoint not landable')
        return False
    node.event('connected_recovery_arrived', endpoint_livo_xy=points[-1].tolist())
    return True
