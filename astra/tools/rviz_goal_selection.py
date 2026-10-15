"""RViz goal selection for the isolated, inspected static-room SITL test.

Clicks never go directly to EGO or PX4. Freeze an estimator-ground reference,
validate the destination, then let the existing flight supervisor own output.
The scan certificate restricts a test destination; it is not a planner map.
"""
import math
import time
import json
import uuid
from rviz_mission import Mission
from planning_envelope import PlanningEnvelope

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from px4_msgs.msg import VehicleStatus, BatteryStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


from flight_recovery import rectangle_covered, spline_height_band


def validate_goal(frame, stamp_ns, now_ns, xyz, origin, height_m, guard,
                  obstacles, obstacle_age_s, allow_origin=False, planning_envelope=None):
    """Return ground-relative XY; RViz's planar Z/yaw do not set flight height."""
    xyz = np.asarray(xyz, dtype=float)
    origin = np.asarray(origin, dtype=float)
    if frame != 'camera_init':
        raise ValueError('Goal frame must be camera_init')
    if (stamp_ns <= 0 or not 0 <= now_ns-stamp_ns <= 2_000_000_000):
        raise ValueError('Goal timestamp is stale or in the future')
    if xyz.shape != (3,) or origin.shape != (3,) or not np.isfinite(np.r_[xyz,origin]).all():
        raise ValueError('Goal/reference contains invalid coordinates')
    if abs(xyz[2]) > 1e-6:
        raise ValueError('Use the 2D Goal Pose tool; height is fixed by the inspected scene')
    if not math.isfinite(height_m) or not .8 <= height_m <= 1.7:
        raise ValueError('Invalid inspected cruise height')
    delta = xyz[:2]-origin[:2]
    distance = np.linalg.norm(delta)
    radius=float(getattr(guard,'metadata',{}).get('interactive_goal_radius_m',3.))
    if not math.isfinite(radius) or not 3. <= radius <= 45.:
        raise ValueError('Invalid inspected goal radius')
    if not (.25 <= distance <= radius or (allow_origin and distance <= .10)):
        raise ValueError(f'Select a goal between 0.25 and {radius:g} m from takeoff')
    boxes = getattr(guard, 'flight_boxes', guard.recovery_boxes)
    inside = getattr(guard, 'inside_flight_region', guard.inside_recovery_region)
    if boxes is None or not inside(delta):
        raise ValueError('Goal is outside the certified transit region' if
                         getattr(guard, 'transit_boxes', None) is not None else
                         'Goal is outside the certified descent region')
    # A neighbourhood reserve covers ground-reference noise and tracking error.
    if not rectangle_covered(boxes,delta-.1,delta+.1):
        raise ValueError('Goal lacks 0.10 m descent-region reserve')
    if planning_envelope is not None and not planning_envelope.contains(xyz[:2]):
        raise ValueError('Goal is outside the shared planning/execution envelope')
    if planning_envelope is not None and not planning_envelope.guide_contains(xyz[:2]):
        raise ValueError('Goal lacks planner detour reserve; select farther inside the safe region')
    # Published maps run at 2 Hz and use the map node's 1 s validity limit.
    # The flight supervisor still checks odom/readiness within 250 ms.
    if not math.isfinite(obstacle_age_s) or not 0 <= obstacle_age_s <= 1.:
        raise ValueError('Local obstacle cloud is stale')
    cloud = np.asarray(obstacles,dtype=float)
    if cloud.ndim != 2 or cloud.shape[1:] != (3,) or not len(cloud) or not np.isfinite(cloud).all():
        raise ValueError('Local obstacle cloud is empty or invalid')
    target = np.array([xyz[0],xyz[1],origin[2]+height_m])
    if np.linalg.norm(cloud-target,axis=1).min() < .2:
        raise ValueError('Goal intersects the inflated live obstacle map')
    return delta.tolist()


class RvizSelection:
    def __init__(self, node, height_m):
        self.node, self.height_m = node, height_m
        self.origin = None
        self.planning_envelope = None
        self.visual_offset = None
        self.enabled = False
        self.mission = Mission()
        self.session = uuid.uuid4().hex
        self.last_request = {}
        self.ui_wall = 0.
        self.flight_deadline = None
        self.stop = False
        self.state = 'Waiting for healthy sensors'
        self.obstacles = None
        self.obstacle_wall = self.obstacle_stamp = 0
        self.battery = None
        self.battery_wall = 0.
        self.selected = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = node.create_publisher(MarkerArray, '/astra/ui/markers', qos)
        self.status_pub = node.create_publisher(String, '/astra/ui/status', qos)
        self.goal_sub = node.create_subscription(PoseStamped, '/astra/ui/goal', self.goal, 10)
        self.stop_sub = node.create_subscription(PointStamped, '/astra/ui/land', self.land, 1)
        self.command_sub = node.create_subscription(String, '/astra/ui/command', self.command, 10)
        self.cloud_sub = node.create_subscription(PointCloud2, '/local_mapping/obstacles', self.cloud, qos_profile_sensor_data)
        from px4_topics import px4_topic
        topic = px4_topic('out', 'battery_status', BatteryStatus, 'x500_0')
        self.battery_sub = node.create_subscription(BatteryStatus, topic, self.on_battery, qos_profile_sensor_data)
        self.timer = node.create_timer(1., self.display)
        self.status_timer = node.create_timer(.5, self.status)

    def on_battery(self, msg):
        self.battery, self.battery_wall = msg, time.monotonic()

    def battery_fault(self):
        if self.battery is None or time.monotonic()-self.battery_wall > 3.:
            return 'mission battery telemetry stale'
        b = self.battery
        if not b.connected or not math.isfinite(b.remaining) or b.remaining < .20 or b.warning >= BatteryStatus.WARNING_LOW:
            return 'mission battery reserve reached'
        return ''

    def cloud(self, msg):
        if msg.header.frame_id != 'camera_init':
            self.obstacles = None
            return
        self.obstacles = point_cloud2.read_points_numpy(msg, field_names=('x','y','z')).copy()
        self.obstacle_wall = time.monotonic()
        self.obstacle_stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

    def check_click(self, msg, allow_origin=False, target_z=None):
        q = msg.pose.position
        now = self.node.get_clock().now().nanoseconds
        age = max(time.monotonic()-self.obstacle_wall, (now-self.obstacle_stamp)/1e9)
        origin = np.array(self.origin, copy=True)
        if target_z is not None:
            origin[2] = target_z-self.height_m
        return validate_goal(msg.header.frame_id,
            msg.header.stamp.sec*10**9+msg.header.stamp.nanosec, now,
            [q.x,q.y,q.z], origin, self.height_m, self.node.mesh_guard,
            self.obstacles, age, allow_origin, self.planning_envelope)

    def goal(self, msg):
        try:
            if not self.enabled or not self.mission.accepting or self.origin is None:
                raise ValueError('Wait for READY; takeoff and landing lock waypoint editing')
            reason = self.node.fault()
            if reason:
                raise ValueError('Flight state not ready: '+reason)
            delta = self.check_click(msg, bool(self.mission.goals))
            self.mission.append(delta)
        except (ValueError, TypeError) as exc:
            self.state = 'REJECTED: '+str(exc)
            self.node.event('rviz_goal_rejected', reason=str(exc))
        else:
            self.selected = [msg.pose.position.x, msg.pose.position.y]
            self.state = 'Waypoint %d queued' % len(self.mission.goals)
            self.node.event('rviz_goal_accepted', clicked_xy=self.selected, offset_xy=delta,
                origin_livo=self.origin.tolist(), return_to_takeoff=False,
                queue_index=len(self.mission.goals)-1, airborne=self.mission.started)
        self.status()

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            now = self.node.get_clock().now().nanoseconds
            if not isinstance(data, dict) or data.get('session') != self.session:
                raise ValueError('Command belongs to a different session')
            stamp, request = data.get('stamp_ns'), data.get('request_id')
            client = data.get('client')
            if (type(stamp) is not int or not 0 <= now-stamp <= 2_000_000_000 or
                    type(request) is not int or not isinstance(client, str) or len(client)>64):
                raise ValueError('Command timestamp or request ID invalid')
            if request <= self.last_request.get(client, -1):
                raise ValueError('Duplicate command')
            if client not in self.last_request and len(self.last_request)>=16:
                raise ValueError('Too many UI clients')
            self.last_request[client] = request
            command = data.get('command')
            if command == 'heartbeat':
                self.ui_wall = time.monotonic()
                return
            if command == 'land':
                self.request_land()
            else:
                if command == 'start':
                    if time.monotonic()-self.ui_wall>2. or self.node.fault() or self.node.initial_alignment_fault() or self.battery_fault():
                        raise ValueError('Sensors or battery not ready')
                    # Recheck the entire edited route against the current live map.
                    for i, delta in enumerate(self.mission.goals):
                        q = PoseStamped(); q.header.frame_id = 'camera_init'
                        q.header.stamp = self.node.get_clock().now().to_msg()
                        q.pose.position.x = float(self.origin[0]+delta[0])
                        q.pose.position.y = float(self.origin[1]+delta[1])
                        self.check_click(q, i>0)
                    if data.get('end_action', 'land') == 'land':
                        from flight_recovery import landing_allowed
                        if not landing_allowed(self.node.mesh_guard, self.mission.goals[-1]):
                            raise ValueError('Last waypoint is transit only; choose return home or hold')
                self.mission.command(command, data)
                self.node.event('rviz_command_accepted', command=command)
                self.state = 'Command: '+command
        except (ValueError, TypeError, KeyError) as exc:
            self.state = 'REJECTED: '+str(exc)
            self.node.event('rviz_command_rejected', reason=str(exc))
        self.status()

    def land(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if msg.header.frame_id == 'camera_init' and stamp>0 and 0 <= self.node.get_clock().now().nanoseconds-stamp <= 2_000_000_000:
            self.request_land()

    def request_land(self):
        from flight_recovery import landing_allowed
        body = getattr(self.node, 'truth_body', None)
        if (self.mission.started and body is not None and
                not landing_allowed(self.node.mesh_guard, body[:2])):
            self.mission.command('return_home', {})
            self.state = 'Returning home through planner'
            self.node.event('rviz_land_redirected_to_return', reason='transit only position')
        else:
            self.stop = True
            self.node.operator_stop_requested = True
            self.mission.close()
            self.node.event('rviz_land_requested')

    def status(self):
        payload = dict(session=self.session, stamp_ns=self.node.get_clock().now().nanoseconds,
            message=self.state, phase=self.node.phase, ready=self.enabled,
            editing=self.mission.accepting, started=self.mission.started, closed=self.mission.closed,
            paused=self.mission.paused, completed=self.mission.completed,
            goals=self.mission.goals, origin=None if self.origin is None else self.origin.tolist(),
            visual_offset=self.visual_offset, height_m=self.height_m,
            goal_radius_m=self.node.mesh_guard.metadata.get('interactive_goal_radius_m',3.),
            alignment_height_m=self.node.mesh_guard.metadata.get('alignment_height_m',1.60),
            scene_name=self.node.mesh_guard.metadata.get('scene_label','Replica office4'),
            end_action=self.mission.end_action, wait_seconds=self.mission.wait_seconds)
        self.status_pub.publish(String(data=json.dumps(payload)))

    def marker(self, identifier, kind):
        m=Marker();m.header.frame_id='camera_init'
        m.header.stamp=self.node.get_clock().now().to_msg()
        m.ns='astra_goal_ui';m.id=identifier;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;return m

    def display(self):
        if self.origin is None:return
        markers=[]
        if self.node.mesh_guard.transit_boxes is not None:
            for i,(lo,hi) in enumerate(self.node.mesh_guard.transit_boxes):
                region=self.marker(100000+i,Marker.CUBE)
                region.pose.position.x=float((lo[0]+hi[0])/2+self.origin[0])
                region.pose.position.y=float((lo[1]+hi[1])/2+self.origin[1])
                region.pose.position.z=float(self.origin[2]+self.height_m)
                region.scale.x=float(hi[0]-lo[0]);region.scale.y=float(hi[1]-lo[1]);region.scale.z=.008
                region.color.b=1.;region.color.g=.55;region.color.a=.12
                markers.append(region)
        certificate=self.node.mesh_guard.metadata.get('recovery_certificate',{})
        if certificate.get('compacted'):
            for i,(lo,hi) in enumerate(self.node.mesh_guard.recovery_boxes):
                region=self.marker(1000+i,Marker.CUBE)
                region.pose.position.x=float((lo[0]+hi[0])/2+self.origin[0])
                region.pose.position.y=float((lo[1]+hi[1])/2+self.origin[1])
                region.scale.x=float(hi[0]-lo[0]);region.scale.y=float(hi[1]-lo[1]);region.scale.z=.008
                region.color.g=.8;region.color.b=.25;region.color.a=.25
                markers.append(region)
        else:
            region=self.marker(1,Marker.CUBE_LIST)
            cell=float(certificate.get('grid_step_xy_m',.05))
            region.scale.x=region.scale.y=cell*.9;region.scale.z=.008
            region.color.g=.8;region.color.b=.25;region.color.a=.25
            for lo,hi in self.node.mesh_guard.recovery_boxes:
                p=Point();p.x=float((lo[0]+hi[0])/2+self.origin[0]);p.y=float((lo[1]+hi[1])/2+self.origin[1]);p.z=0.
                region.points.append(p)
            markers.append(region)
        # DELETEALL only in this marker namespace/topic avoids leftover queue labels.
        clear=self.marker(0,Marker.SPHERE);clear.action=Marker.DELETEALL
        markers.insert(0,clear)
        for i,delta in enumerate(self.mission.goals):
            goal=self.marker(10+2*i,Marker.SPHERE)
            goal.pose.position.x=float(self.origin[0]+delta[0]);goal.pose.position.y=float(self.origin[1]+delta[1])
            goal.pose.position.z=float(self.origin[2]+self.height_m)
            goal.scale.x=goal.scale.y=goal.scale.z=.12
            goal.color.r=1.;goal.color.g=.8;goal.color.a=1.
            if i<self.mission.completed:goal.color.r=.2;goal.color.g=1.
            markers.append(goal)
            label=self.marker(11+2*i,Marker.TEXT_VIEW_FACING)
            label.pose.position=goal.pose.position;label.pose.position.z+=.18
            label.scale.z=.14;label.color.r=label.color.g=label.color.b=label.color.a=1.
            label.text=str(i+1);markers.append(label)
        self.pub.publish(MarkerArray(markers=markers))

    def wait(self, timeout_s):
        node=self.node;deadline=time.monotonic()+timeout_s if timeout_s else float('inf')
        samples=[];last_stamp=None
        node.phase='rviz_goal_selection'
        node.event('rviz_goal_wait', timeout_s=timeout_s, cruise_height_m=self.height_m,
            goal_topic='/astra/ui/goal', command_topic='/astra/ui/command')
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if self.stop:raise RuntimeError('Operator cancelled RViz mission')
            reason=node.fault() or node.initial_alignment_fault() or self.battery_fault()
            if reason or node.status.arming_state==VehicleStatus.ARMING_STATE_ARMED:
                self.enabled=self.mission.accepting=False;samples=[]
                self.state='Not ready: '+(reason or 'vehicle already armed')
                if self.mission.started:raise RuntimeError('Readiness lost after START')
                continue
            stamp=node.odom.header.stamp.sec*10**9+node.odom.header.stamp.nanosec
            p=node.odom.pose.pose.position
            if stamp!=last_stamp:
                samples.append((stamp,[p.x,p.y,p.z]));samples=samples[-25:];last_stamp=stamp
            if self.origin is None:
                if len(samples)<15 or samples[-1][0]-samples[0][0]<1_000_000_000:continue
                positions=np.array([x[1] for x in samples])
                if np.ptp(positions,axis=0).max()>.02:continue
                self.origin=np.median(positions,axis=0)
                self.planning_envelope=PlanningEnvelope(node.mesh_guard.flight_boxes,
                    node.mesh_guard.bounds,self.origin[:2],
                    z_bounds=spline_height_band(node.mesh_guard,self.origin[2]),
                    connected_to_origin=node.mesh_guard.metadata.get('whole_map_selection',False))
                # Truth is used ONLY to position the optional background illustration.
                self.visual_offset=(self.origin-np.asarray(node.truth_body)).tolist()
                node.event('rviz_ground_reference_frozen',origin_livo=self.origin.tolist(),
                    source='FAST-LIVO2 ground measurements; no truth calibration for control')
            if np.linalg.norm(np.array([p.x,p.y,p.z])-self.origin)>.02:
                self.enabled=self.mission.accepting=False
                self.state='Ground reference moved: restart session'
                if self.mission.started:raise RuntimeError(self.state)
                continue
            if self.mission.started:
                self.enabled=False
                self.state='Starting mission'
                self.status()
                return self.mission.goals
            if not self.enabled:
                self.enabled=self.mission.accepting=True
                self.state='READY: add waypoints, then press Start'
                node.get_logger().info(self.state)
                self.status();self.display()
        raise RuntimeError('RViz mission editing timed out without arming')

    def open_airborne(self):
        self.enabled=self.mission.accepting=True
        self.flight_deadline=time.monotonic()+8.*(100.+120.)
        self.state='Flying: additional waypoints join the queue'
        self.status()

    def flight_fault(self):
        if self.mission.started and not self.mission.closed:
            if self.flight_deadline is not None and time.monotonic()>self.flight_deadline:
                return 'mission session duration exceeded'
            if time.monotonic()-self.ui_wall>2.:
                return 'mission control panel heartbeat lost'
            return self.battery_fault()
        return ''
