#!/usr/bin/env python3
"""Bounded obstacle cloud and localization gate for simulation planning."""

import math
import signal
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header


def inflate_cells(cells, offsets):
    """Deduplicate integer voxels using scalar keys instead of sorting XYZ rows."""
    if not len(cells):
        return np.empty((0, 3), dtype=np.int64)
    cells = np.asarray(cells, dtype=np.int64)
    offsets = np.asarray(offsets, dtype=np.int64)
    lower = cells.min(axis=0) + offsets.min(axis=0)
    dimensions = cells.max(axis=0) + offsets.max(axis=0) - lower + 1
    volume = math.prod(int(d) for d in dimensions)
    if volume > np.iinfo(np.int64).max:
        raise ValueError('voxel extent exceeds integer key range')
    strides = np.asarray((dimensions[1] * dimensions[2], dimensions[2], 1))
    base_keys = (cells - lower) @ strides
    offset_keys = offsets @ strides
    if volume <= 8_000_000:
        # Exact set union in at most 8 MB; avoid sorting millions of repeated
        # keys from dense floor/obstacle scans. Chunk temporary indexing arrays.
        occupied = np.zeros(volume, dtype=bool)
        for begin in range(0, len(cells), 4096):
            occupied[base_keys[begin:begin+4096, None] + offset_keys[None, :]] = True
        keys = np.flatnonzero(occupied)
    else:
        # Sparse, widely separated coordinates must not allocate by extent.
        keys = np.unique(base_keys[:, None] + offset_keys[None, :])
    return np.column_stack(np.unravel_index(keys, tuple(dimensions))) + lower


def static_descent_obstacles(cells, offsets, voxel_size, minimum_obstacle_z, ceiling_z,
                             column_offsets=None,maximum_obstacle_z=None):
    """Conservative columns from observed objects only; never extrude the floor.

    Used for isolated static-room tests that recover by landing at current XY.
    The stored 3D occupied map remains unchanged. No scan mesh or truth enters.
    """
    inflated=inflate_cells(cells,offsets)
    z=(cells[:,2]+.5)*voxel_size
    maximum_obstacle_z=ceiling_z if maximum_obstacle_z is None else maximum_obstacle_z
    elevated=cells[(z>=minimum_obstacle_z) & (z<=maximum_obstacle_z)]
    if not len(elevated):return inflated
    object_xy=np.unique(inflate_cells(elevated,offsets if column_offsets is None else column_offsets)[:,:2],axis=0)
    heights=np.arange(0,int(math.ceil(ceiling_z/voxel_size)),dtype=np.int64)
    columns=np.column_stack((np.repeat(object_xy,len(heights),axis=0),
                             np.tile(heights,len(object_xy))))
    return inflate_cells(np.vstack((inflated,columns)),np.zeros((1,3),dtype=np.int64))


class LocalMappingGuard(Node):
    def __init__(self):
        super().__init__('local_mapping_guard')
        self.declare_parameter('voxel_size_m', 0.20)
        self.declare_parameter('radius_m', 5.0)
        self.declare_parameter('obstacle_inflation_m', 0.65)
        self.declare_parameter('max_odom_step_m', 0.12)
        self.declare_parameter('max_odom_age_sec', 0.5)
        self.declare_parameter('max_cloud_age_sec', 1.0)
        self.declare_parameter('voxel_ttl_sec', 3.0)
        self.declare_parameter('recovery_odom_count', 10)
        self.declare_parameter('min_map_voxels', 20)
        self.declare_parameter('cloud_topic', '/fast_livo2/cloud_registered_full')
        self.declare_parameter('self_filter_radius_m', 0.0)
        self.declare_parameter('max_map_voxels', 5000)
        self.declare_parameter('static_descent_columns', False)
        self.declare_parameter('column_obstacle_min_z_m', .15)
        self.declare_parameter('column_ceiling_m', 1.5)
        self.declare_parameter('column_obstacle_max_z_m', -1.0)
        self.declare_parameter('descent_column_inflation_m', 0.0)

        self.voxel_size = float(self.get_parameter('voxel_size_m').value)
        self.radius = float(self.get_parameter('radius_m').value)
        self.inflation = float(self.get_parameter('obstacle_inflation_m').value)
        self.max_step = float(self.get_parameter('max_odom_step_m').value)
        self.max_odom_age = float(self.get_parameter('max_odom_age_sec').value)
        self.max_cloud_age = float(self.get_parameter('max_cloud_age_sec').value)
        self.voxel_ttl = float(self.get_parameter('voxel_ttl_sec').value)
        self.recovery_count = int(self.get_parameter('recovery_odom_count').value)
        self.min_voxels = int(self.get_parameter('min_map_voxels').value)
        self.max_voxels = int(self.get_parameter('max_map_voxels').value)
        self.self_filter_radius = float(self.get_parameter('self_filter_radius_m').value)
        self.static_columns = bool(self.get_parameter('static_descent_columns').value)
        self.column_min_z = float(self.get_parameter('column_obstacle_min_z_m').value)
        self.column_ceiling = float(self.get_parameter('column_ceiling_m').value)
        self.column_max_z = float(self.get_parameter('column_obstacle_max_z_m').value)
        if self.column_max_z == -1.0:self.column_max_z=self.column_ceiling
        if not math.isfinite(self.column_max_z) or not self.column_ceiling<=self.column_max_z<=5.:
            raise ValueError('Invalid observed-object height range for descent columns')
        self.column_inflation = float(self.get_parameter('descent_column_inflation_m').value)
        if not math.isfinite(self.column_inflation) or not 0<=self.column_inflation<=1.5:
            raise ValueError('Invalid descent-column horizontal inflation')
        if self.static_columns and (not all(math.isfinite(x) for x in (self.column_min_z,self.column_ceiling))
                or not 0<=self.column_min_z<self.column_ceiling<=2.3):
            raise ValueError('Invalid static descent column heights in camera_init')
        if not math.isfinite(self.self_filter_radius) or self.self_filter_radius < 0:
            raise ValueError('self filter radius must be finite and non-negative')
        if min(self.voxel_size, self.radius, self.max_step, self.voxel_ttl) <= 0:
            raise ValueError('voxel size, radius, odom step and voxel TTL must be positive')
        if self.inflation < 0 or self.max_voxels < 1 or self.recovery_count < 1:
            raise ValueError('invalid inflation, map capacity or recovery count')

        self.voxels = {}
        self.last_odom = None
        self.last_odom_ns = None
        self.last_odom_wall = None
        self.last_cloud_ns = None
        self.last_cloud_wall = None
        self.position = None
        self.frame_id = None
        self.good_odom = 0
        self.reason = 'waiting for odometry'
        self.ready = False
        self.offsets = self._inflation_offsets()
        self.column_offsets=None
        if self.column_inflation>self.inflation:
            n=int(math.ceil(self.column_inflation/self.voxel_size))
            xy=np.stack(np.meshgrid(np.arange(-n,n+1),np.arange(-n,n+1),indexing='ij'),axis=-1).reshape(-1,2)
            xy=xy[np.linalg.norm(xy*self.voxel_size,axis=1)<=self.column_inflation+1e-9]
            self.column_offsets=np.column_stack((xy,np.zeros(len(xy),dtype=int)))
        self.map_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='local_map')
        self.map_future = None
        self.map_generation = 0
        self.map_build_ms = 0.0
        self.map_results_discarded = 0
        self.last_map_ns = None
        self.last_map_wall = None
        self.last_odom_interval_ms = 0.0
        self.last_health_wall = None
        self.max_health_interval_ms = 0.0
        self.last_invalidation_log_wall = 0.0
        self.invalidation_counts = {}

        self.create_subscription(Odometry, '/fast_livo2/odom', self.on_odom, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter('cloud_topic').value),
                                 self.on_cloud, 2)
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, '/local_mapping/ready', ready_qos)
        self.occupied_pub = self.create_publisher(PointCloud2, '/local_mapping/occupied', 2)
        self.obstacles_pub = self.create_publisher(PointCloud2, '/local_mapping/obstacles', 2)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/local_mapping/diagnostics', 10)
        self.ready_pub.publish(Bool(data=False))
        wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.check_health, clock=wall_clock)
        self.create_timer(0.5, self.publish_map, clock=wall_clock)

    def invalidate(self, reason):
        now_wall = time.monotonic()
        self.invalidation_counts[reason] = self.invalidation_counts.get(reason, 0) + 1
        # Keep the safety transition immediate, but rate-limit diagnostics so a
        # bad sensor stream cannot make logging the source of more scheduling
        # pressure.
        if (reason != getattr(self, 'last_invalidation_reason', None) or
                now_wall - self.last_invalidation_log_wall >= 1.0):
            cloud_age = ('unknown' if self.last_cloud_wall is None else
                         '%.1f' % ((now_wall - self.last_cloud_wall) * 1000.0))
            self.get_logger().warning(
                f'Readiness invalidated: {reason} (voxels={len(self.voxels)}, '
                f'good_odom={self.good_odom}, map_build_ms={self.map_build_ms:.1f}, '
                f'last_cloud_age_ms={cloud_age})')
            self.last_invalidation_log_wall = now_wall
            self.last_invalidation_reason = reason
        self.map_generation += 1
        self.good_odom = 0
        self.voxels.clear()
        self.last_cloud_ns = None
        self.last_cloud_wall = None
        self.last_map_ns = None
        self.last_map_wall = None
        self.reason = reason
        self.ready = False
        self.ready_pub.publish(Bool(data=False))

    def health_reason(self, now_ns, check_published_map=True):
        now_wall = time.monotonic()
        if self.good_odom < self.recovery_count:
            return self.reason if self.good_odom == 0 else 'recovering odometry'
        if (self.last_odom_ns is None or self.last_odom_wall is None or
                now_ns - self.last_odom_ns > self.max_odom_age * 1e9 or
                now_wall - self.last_odom_wall > self.max_odom_age):
            return 'stale odometry'
        if (self.last_cloud_ns is None or self.last_cloud_wall is None or
                now_ns - self.last_cloud_ns > self.max_cloud_age * 1e9 or
                now_wall - self.last_cloud_wall > self.max_cloud_age):
            return 'stale obstacle cloud'
        if len(self.voxels) < self.min_voxels:
            return 'insufficient local obstacle points'
        if check_published_map and self.ready and (self.last_map_ns is None or self.last_map_wall is None or
                now_ns - self.last_map_ns > self.max_cloud_age * 1e9 or
                now_wall - self.last_map_wall > self.max_cloud_age):
            return 'stale published obstacle map'
        return ''

    def check_health(self):
        now_wall = time.monotonic()
        if self.last_health_wall is not None:
            self.max_health_interval_ms = max(
                self.max_health_interval_ms, (now_wall - self.last_health_wall) * 1000)
        self.last_health_wall = now_wall
        # A completed fresh snapshot may replace the old map at its expiry
        # boundary. Validate and publish it before judging the old publication;
        # stale inputs/results and in-flight work still fail closed below.
        self.finish_map()
        if self.ready:
            reason = self.health_reason(self.get_clock().now().nanoseconds)
            if reason:
                self.invalidate(reason)
        # Readiness is also a liveness heartbeat, independent of map publication.
        self.ready_pub.publish(Bool(data=self.ready))

    def _inflation_offsets(self):
        cells = int(math.ceil(self.inflation / self.voxel_size))
        return np.asarray([
            (x, y, z) for x in range(-cells, cells + 1)
            for y in range(-cells, cells + 1)
            for z in range(-cells, cells + 1)
            if math.sqrt(x * x + y * y + z * z) * self.voxel_size
            <= self.inflation + 1e-9
        ], dtype=np.int32)

    def on_odom(self, msg):
        now_wall = time.monotonic()
        if self.last_odom_wall is not None:
            self.last_odom_interval_ms = (now_wall - self.last_odom_wall) * 1000
        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        p = msg.pose.pose.position
        position = np.asarray((p.x, p.y, p.z), dtype=np.float64)
        if not np.isfinite(position).all() or stamp_ns <= 0:
            self.invalidate('non-finite or unstamped odometry')
            return
        if self.frame_id is not None and msg.header.frame_id != self.frame_id:
            self.invalidate('odometry frame changed')
        if self.last_odom_ns is not None:
            step = np.linalg.norm(position - self.last_odom)
            if stamp_ns <= self.last_odom_ns or step > self.max_step:
                self.invalidate('odometry timestamp regression or jump')
            else:
                self.good_odom = min(self.good_odom + 1, self.recovery_count)
        else:
            self.good_odom = 1
        self.last_odom = position
        self.position = position
        self.last_odom_ns = stamp_ns
        self.last_odom_wall = now_wall
        self.frame_id = msg.header.frame_id

    def on_cloud(self, msg):
        if self.good_odom < self.recovery_count or self.position is None:
            return
        if msg.width * msg.height == 0 or not {'x', 'y', 'z'}.issubset(
                field.name for field in msg.fields):
            return
        if msg.header.frame_id != self.frame_id:
            self.invalidate('cloud and odometry frames differ')
            return
        stamp_ns = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= 0 or abs(stamp_ns - self.last_odom_ns) > 1_000_000_000:
            self.invalidate('cloud timestamp outside odometry window')
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        if not len(points):
            return
        points = np.asarray(points, dtype=np.float64)
        points = points[np.isfinite(points).all(axis=1)]
        distances = np.linalg.norm(points - self.position, axis=1)
        # Configure only from the actual airframe envelope. The sensor can see
        # its own propellers in SITL; those returns must not become obstacles.
        points = points[(distances <= self.radius) & (distances >= self.self_filter_radius)]
        cells = np.unique(np.floor(points / self.voxel_size).astype(np.int32), axis=0)
        for cell in cells:
            self.voxels[tuple(cell)] = stamp_ns
        self.last_cloud_ns = stamp_ns
        self.last_cloud_wall = time.monotonic()

    def publish_map(self):
        self.finish_map()
        now_ns = self.get_clock().now().nanoseconds
        if self.position is not None and self.voxels:
            keys = list(self.voxels)
            cells = np.asarray(keys, dtype=np.int64)
            seen = np.fromiter(self.voxels.values(), dtype=np.int64)
            distances_sq = np.sum((cells * self.voxel_size - self.position) ** 2, axis=1)
            expired = (now_ns - seen > self.voxel_ttl * 1e9) | (distances_sq > self.radius ** 2)
            for index in np.flatnonzero(expired):
                del self.voxels[keys[index]]
        if len(self.voxels) > self.max_voxels:
            self.invalidate('local map capacity exceeded')

        reason = self.health_reason(now_ns)
        if reason and self.ready:
            self.invalidate(reason)
        self.reason = reason or ('ready' if self.ready else 'building local map')

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'local_mapping_guard'
        status.level = DiagnosticStatus.OK if self.ready else DiagnosticStatus.WARN
        status.message = self.reason
        status.values = [KeyValue(key='voxel_count', value=str(len(self.voxels))),
                         KeyValue(key='static_descent_columns', value=str(self.static_columns)),
                         KeyValue(key='descent_column_inflation_m', value=str(max(self.inflation,self.column_inflation))),
                         KeyValue(key='consecutive_good_odom', value=str(self.good_odom)),
                         KeyValue(key='map_build_ms', value='%.3f' % self.map_build_ms),
                         KeyValue(key='map_build_pending', value=str(self.map_future is not None)),
                         KeyValue(key='map_results_discarded', value=str(self.map_results_discarded)),
                         KeyValue(key='odom_receipt_interval_ms', value='%.3f' % self.last_odom_interval_ms),
                         KeyValue(key='max_health_interval_ms', value='%.3f' % self.max_health_interval_ms)]
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)
        if reason or self.map_future is not None:
            return
        cells = np.asarray(list(self.voxels), dtype=np.int32)
        header = Header()
        header.stamp = rclpy.time.Time(nanoseconds=self.last_cloud_ns).to_msg()
        header.frame_id = self.frame_id
        # A single immutable snapshot keeps map work from delaying odometry and health callbacks.
        self.map_future_generation = self.map_generation
        self.map_snapshot_ns = self.last_cloud_ns
        self.map_started_wall = time.monotonic()
        self.map_future = self.map_worker.submit(self.build_clouds, cells, header)

    def build_clouds(self, cells, header):
        inflated = (static_descent_obstacles(cells,self.offsets,self.voxel_size,self.column_min_z,self.column_ceiling,
                                            self.column_offsets,self.column_max_z)
                    if self.static_columns else inflate_cells(cells, self.offsets))
        occupied = point_cloud2.create_cloud_xyz32(
            header, ((cells + 0.5) * self.voxel_size).astype(np.float32))
        obstacles = point_cloud2.create_cloud_xyz32(
            header, ((inflated + 0.5) * self.voxel_size).astype(np.float32))
        return occupied, obstacles

    def finish_map(self):
        if self.map_future is None or not self.map_future.done():
            return
        future, self.map_future = self.map_future, None
        now_ns = self.get_clock().now().nanoseconds
        self.map_build_ms = (time.monotonic() - self.map_started_wall) * 1000
        input_reason = self.health_reason(now_ns, check_published_map=False)
        if (self.map_future_generation != self.map_generation or input_reason or
                now_ns - self.map_snapshot_ns > self.max_cloud_age * 1e9 or
                time.monotonic() - self.map_started_wall > self.max_cloud_age):
            self.map_results_discarded += 1
            self.get_logger().warning(
                f'Discarded local map result: build_ms={self.map_build_ms:.1f}, '
                f'voxels={len(self.voxels)}, generation={self.map_future_generation}/'
                f'{self.map_generation}, reason={input_reason or "stale result"}')
            return
        try:
            occupied, obstacles = future.result()
        except Exception as exc:
            self.get_logger().error('Local map build failed: %s' % exc)
            self.invalidate('local map build failed')
            return
        self.occupied_pub.publish(occupied)
        self.obstacles_pub.publish(obstacles)
        self.last_map_ns = self.map_snapshot_ns
        self.last_map_wall = time.monotonic()
        self.ready = True
        self.reason = 'ready'
        self.get_logger().debug(
            f'Local map ready: voxels={len(self.voxels)} build_ms={self.map_build_ms:.1f}')

    def destroy_node(self):
        self.map_worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = LocalMappingGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
