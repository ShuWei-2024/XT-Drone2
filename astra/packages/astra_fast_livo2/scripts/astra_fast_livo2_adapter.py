#!/usr/bin/env python3
"""ROS 2 input adapter and timing diagnostics for the FAST-LIVO2 estimator.

The node does not estimate pose and does not publish FAST-LIVO2 odometry. It
only normalizes the Gazebo sensor contract and exposes both an enriched
PointCloud2 and a ROS 2 equivalent of Livox CustomMsg.
"""

from collections import Counter, deque
import csv
import signal
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from astra_fast_livo2_msgs.msg import LivoxCustomMsg, LivoxCustomPoint
from input_callback_audit import InputCallbackAudit, discard_expired_lidar_head


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class FastLivo2Adapter(Node):
    def __init__(self):
        super().__init__('astra_fast_livo2_adapter')

        self.declare_parameter('lidar_topic', '/sensors/lidar/points')
        self.declare_parameter('image_topic', '/sensors/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/sensors/camera/color/camera_info')
        self.declare_parameter('imu_topic', '/sensors/imu/data')
        self.declare_parameter('output_lidar_topic', '/fast_livo2/input/lidar')
        self.declare_parameter('output_livox_topic', '/fast_livo2/input/livox_custom')
        self.declare_parameter('output_image_topic', '/fast_livo2/input/image')
        self.declare_parameter('output_camera_info_topic', '/fast_livo2/input/camera_info')
        self.declare_parameter('output_imu_topic', '/fast_livo2/input/imu')
        self.declare_parameter('synced_lidar_topic', '/fast_livo2/sync/lidar')
        self.declare_parameter('synced_image_topic', '/fast_livo2/sync/image')
        self.declare_parameter('synced_imu_topic', '/fast_livo2/sync/imu')
        self.declare_parameter('diagnostics_topic', '/fast_livo2/diagnostics')
        self.declare_parameter('lidar_frame_id', 'mid360_scan_frame')
        self.declare_parameter('image_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('scan_period_sec', 0.1)
        self.declare_parameter('point_time_mode', 'synthetic_scan')
        self.declare_parameter('publish_livox_custom', True)
        self.declare_parameter('publish_unsynced_image', True)
        self.declare_parameter('reliable_image_subscription', False)
        self.declare_parameter('reliable_lidar_subscription', False)
        self.declare_parameter('reliable_imu_subscription', False)
        self.declare_parameter('reliable_synced_inputs', False)
        self.declare_parameter('max_lidar_image_delta_sec', 0.03)
        self.declare_parameter('max_lidar_imu_delta_sec', 0.02)
        self.declare_parameter('diagnostic_period_sec', 1.0)
        self.declare_parameter('input_audit_csv', '')
        self.declare_parameter('trace_sensor_callbacks', False)
        self.callback_groups = {
            name: MutuallyExclusiveCallbackGroup()
            for name in ('lidar', 'image', 'imu', 'camera_info', 'diagnostics', 'sync')
        }

        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.image_frame_id = self.get_parameter('image_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.scan_period_sec = float(self.get_parameter('scan_period_sec').value)
        self.point_time_mode = self.get_parameter('point_time_mode').value
        if self.point_time_mode not in ('synthetic_scan', 'snapshot'):
            raise ValueError('point_time_mode must be synthetic_scan or snapshot')
        if not np.isfinite(self.scan_period_sec) or self.scan_period_sec <= 0:
            raise ValueError('scan_period_sec must be finite and positive')
        self.scan_duration_sec = (0.0 if self.point_time_mode == 'snapshot'
                                  else self.scan_period_sec)
        self.publish_livox_custom = bool(self.get_parameter('publish_livox_custom').value)
        self.publish_unsynced_image = bool(self.get_parameter('publish_unsynced_image').value)
        self.reliable_image_subscription = bool(self.get_parameter('reliable_image_subscription').value)
        self.reliable_synced_inputs = bool(self.get_parameter('reliable_synced_inputs').value)
        self.max_lidar_image_delta = float(self.get_parameter('max_lidar_image_delta_sec').value)
        self.max_lidar_imu_delta = float(self.get_parameter('max_lidar_imu_delta_sec').value)

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.pub_lidar = self.create_publisher(
            PointCloud2, self.get_parameter('output_lidar_topic').value, pub_qos)
        self.pub_livox = None
        if self.publish_livox_custom:
            self.pub_livox = self.create_publisher(
                LivoxCustomMsg, self.get_parameter('output_livox_topic').value, pub_qos)
        self.pub_image = None
        if self.publish_unsynced_image:
            self.pub_image = self.create_publisher(
                Image, self.get_parameter('output_image_topic').value, pub_qos)
        self.pub_camera_info = self.create_publisher(
            CameraInfo, self.get_parameter('output_camera_info_topic').value, pub_qos)
        self.pub_imu = self.create_publisher(
            Imu, self.get_parameter('output_imu_topic').value, pub_qos)
        sync_qos = (QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    if self.reliable_synced_inputs else pub_qos)
        self.pub_sync_lidar = self.create_publisher(
            PointCloud2, self.get_parameter('synced_lidar_topic').value, sync_qos)
        self.pub_sync_image = self.create_publisher(
            Image, self.get_parameter('synced_image_topic').value, sync_qos)
        self.pub_sync_imu = self.create_publisher(
            Imu, self.get_parameter('synced_imu_topic').value, pub_qos)
        self.pub_diagnostics = self.create_publisher(
            DiagnosticArray, self.get_parameter('diagnostics_topic').value, 10)

        self.histories = {name: deque(maxlen=200) for name in ('lidar', 'image', 'imu', 'camera_info')}
        self.history_lock = threading.Lock()
        self.last_stamp = {}
        self.counts = {name: 0 for name in self.histories}
        self.last_warning = {}
        self.camera_info_received = False
        self.stopping = False
        self.sync_lock = threading.Lock()
        self.sync_lidar = deque()
        self.sync_images = deque(maxlen=24)
        self.sync_imus = deque(maxlen=400)
        self.sync_counts = Counter()
        self.sync_deltas_ms = deque(maxlen=200)
        self.sync_wait_ms = deque(maxlen=200)
        self.last_sync_lidar_stamp = None
        self.audit_lock = threading.Lock()
        self.audit_rows = deque(maxlen=100000)
        self.audit_stop = threading.Event()
        self.audit_thread = None
        self.callback_audit = None
        audit_path = str(self.get_parameter('input_audit_csv').value)
        if audit_path:
            self.audit_stream = open(audit_path, 'w', newline='')
            self.audit_writer = csv.writer(self.audit_stream)
            self.audit_writer.writerow(['event', 'stamp_ns', 'wall_ns', 'clock_ns',
                                        'image_ns', 'imu_ns', 'detail'])
            self.audit_thread = threading.Thread(target=self._write_audit, daemon=True)
            self.audit_thread.start()
            if self.get_parameter('trace_sensor_callbacks').value:
                self.callback_audit = InputCallbackAudit(self._audit)
        def traced(callback, label):
            return self.callback_audit.subscription(callback, label) if self.callback_audit else callback
        lidar_qos = (QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                     if self.get_parameter('reliable_lidar_subscription').value
                     else qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            traced(self.lidar_callback, 'lidar'), lidar_qos,
            callback_group=self.callback_groups['lidar'])
        image_qos = (QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                     if self.reliable_image_subscription else qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            traced(self.image_callback, 'image'), image_qos,
            callback_group=self.callback_groups['image'])
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            traced(self.camera_info_callback, 'camera_info'), qos_profile_sensor_data,
            callback_group=self.callback_groups['camera_info'])
        imu_qos = (QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
                   if self.get_parameter('reliable_imu_subscription').value
                   else qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            traced(self.imu_callback, 'imu'), imu_qos,
            callback_group=self.callback_groups['imu'])
        self.timer = self.create_timer(
            float(self.get_parameter('diagnostic_period_sec').value), self.publish_diagnostics,
            callback_group=self.callback_groups['diagnostics'],
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.sync_timer = self.create_timer(
            0.01, self.callback_audit.timer(self.process_sync, 'sync') if self.callback_audit else self.process_sync,
            callback_group=self.callback_groups['sync'],
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info('FAST-LIVO2 adapter started; pose estimation is not implemented in this node')
        self.get_logger().info(
            f'LiDAR point time mode: {self.point_time_mode}; '
            f'{self.scan_duration_sec:.3f} s measurement span; '
            'no Livox timing model is claimed')

    def _audit(self, event, stamp=0, image=0, imu=0, detail=''):
        if self.audit_thread is None:
            return
        row = (event, stamp, time.monotonic_ns(), self.get_clock().now().nanoseconds,
               image, imu, detail)
        with self.audit_lock:
            self.audit_rows.append(row)

    def _write_audit(self):
        while not self.audit_stop.wait(0.5):
            self._flush_audit()
        self._flush_audit()

    def _flush_audit(self):
        with self.audit_lock:
            rows = tuple(self.audit_rows)
            self.audit_rows.clear()
        self.audit_writer.writerows(rows)
        self.audit_stream.flush()

    def destroy_node(self):
        if self.callback_audit is not None:
            self.callback_audit.close()
        if self.audit_thread is not None:
            self.audit_stop.set()
            self.audit_thread.join()
            self.audit_stream.close()
        return super().destroy_node()

    def _record(self, name, msg, expected_frame=None):
        stamp = stamp_ns(msg)
        with self.history_lock:
            previous = self.last_stamp.get(name)
            if previous is None or stamp > previous:
                self.last_stamp[name] = stamp
                self.histories[name].append(stamp)
                self.counts[name] += 1
        if previous is not None and stamp <= previous:
            self._warn_periodic(name + '_backward',
                                '%s timestamp is not strictly increasing: %d <= %d; dropping sample' %
                                (name, stamp, previous))
            return None
        if expected_frame and msg.header.frame_id != expected_frame:
            self._warn_periodic(name + '_frame',
                                '%s frame_id=%s, expected %s' %
                                (name, msg.header.frame_id, expected_frame))
        return stamp

    def _warn_periodic(self, key, message, period=5.0):
        now = time.monotonic()
        if now - self.last_warning.get(key, 0.0) >= period:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def lidar_callback(self, msg):
        if self.stopping:
            return
        self._audit('lidar_enter', stamp_ns(msg))
        lidar_stamp = self._record('lidar', msg, self.lidar_frame_id)
        if lidar_stamp is None:
            return
        field_names = {field.name for field in msg.fields}
        required = {'x', 'y', 'z'}
        if not required.issubset(field_names):
            self._warn_periodic('lidar_fields',
                                'PointCloud2 is missing x/y/z fields: %s' % sorted(field_names))
            return

        raw = point_cloud2.read_points(msg, skip_nans=False)
        valid = np.isfinite(raw['x']) & np.isfinite(raw['y']) & np.isfinite(raw['z'])
        indices = np.flatnonzero(valid)
        width = max(int(msg.width), 1)
        height = max(int(msg.height), 1)
        # Keep original organized-cloud indices so synthetic point time stays
        # monotonic even when invalid points are removed.
        offset_ns = (indices / max(width * height - 1, 1) *
                     self.scan_duration_sec * 1e9).astype(np.uint32)

        header = msg.header
        header.frame_id = self.lidar_frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
        ]
        dtype = point_cloud2.dtype_from_fields(fields)
        points = np.empty(len(indices), dtype=dtype)
        for axis in ('x', 'y', 'z'):
            points[axis] = raw[axis][valid]
        points['intensity'] = raw['intensity'][valid] if 'intensity' in field_names else 0.0
        points['time'] = offset_ns / 1e3
        if 'ring' in field_names:
            points['ring'] = np.clip(raw['ring'][valid], 0, 255)
        else:
            points['ring'] = np.minimum(indices // width if height > 1 else 0, 255)
        enriched = point_cloud2.create_cloud(header, fields, points)
        self.pub_lidar.publish(enriched)
        with self.sync_lock:
            self.sync_counts['total_lidar'] += 1
            if len(self.sync_lidar) >= 8:
                self.sync_lidar.popleft()
                self.sync_counts['queue_overflow'] += 1
            self.sync_lidar.append((lidar_stamp, time.monotonic(), enriched))
            self._audit('lidar_buffered', lidar_stamp)
        if self.pub_livox is not None:
            custom_points = [LivoxCustomPoint(
                offset_time=int(offset), x=float(point['x']), y=float(point['y']),
                z=float(point['z']),
                reflectivity=max(0, min(255, int(round(float(point['intensity']))))),
                tag=0x10, line=int(point['ring']))
                for point, offset in zip(points, offset_ns)]
            custom = LivoxCustomMsg()
            custom.header = header
            custom.timebase = lidar_stamp
            custom.point_num = len(custom_points)
            custom.lidar_id = 0
            custom.rsvd = [0, 0, 0]
            custom.points = custom_points
            self.pub_livox.publish(custom)

        image_delta = self._nearest_delta('image', lidar_stamp)
        imu_delta = self._nearest_delta('imu', lidar_stamp)
        if image_delta is not None and image_delta > self.max_lidar_image_delta:
            self._warn_periodic('image_sync',
                                'LiDAR/RGB timestamp delta %.3f s exceeds %.3f s' %
                                (image_delta, self.max_lidar_image_delta))
        if imu_delta is not None and imu_delta > self.max_lidar_imu_delta:
            self._warn_periodic('imu_sync',
                                'LiDAR/IMU timestamp delta %.3f s exceeds %.3f s' %
                                (imu_delta, self.max_lidar_imu_delta))

    def image_callback(self, msg):
        if self.stopping:
            return
        self._audit('image_enter', stamp_ns(msg))
        image_stamp = self._record('image', msg, self.image_frame_id)
        if image_stamp is not None:
            with self.sync_lock:
                self.sync_images.append((image_stamp, msg))
                self._audit('image_buffered', image_stamp)
            if self.pub_image is not None:
                self.pub_image.publish(msg)

    def camera_info_callback(self, msg):
        if self.stopping:
            return
        if self._record('camera_info', msg, self.image_frame_id) is None:
            return
        self.pub_camera_info.publish(msg)
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info(
                f'CameraInfo: {msg.width}x{msg.height} model={msg.distortion_model} '
                f'fx={msg.k[0]:.6f} fy={msg.k[4]:.6f} cx={msg.k[2]:.6f} '
                f'cy={msg.k[5]:.6f} D={list(msg.d)}')

    def imu_callback(self, msg):
        if self.stopping:
            return
        self._audit('imu_enter', stamp_ns(msg))
        imu_stamp = self._record('imu', msg, self.imu_frame_id)
        if imu_stamp is not None:
            with self.sync_lock:
                self.sync_imus.append((imu_stamp, msg))
                self._audit('imu_buffered', imu_stamp)
            if self.callback_audit: self._audit('imu_raw_publish_start', imu_stamp)
            self.pub_imu.publish(msg)
            if self.callback_audit: self._audit('imu_raw_publish_end', imu_stamp)
            self.pub_sync_imu.publish(msg)
            if self.callback_audit: self._audit('imu_sync_publish_end', imu_stamp)

    def process_sync(self):
        if self.stopping:
            return
        self._audit('sync_enter')
        with self.sync_lock:
            if not self.sync_lidar:
                self._audit('sync_empty')
                return
            stamp, received, cloud = self.sync_lidar[0]
            self._audit('sync_locked', stamp)
            scan_end = stamp + int(self.scan_duration_sec * 1e9)
            watermark = stamp + int(self.max_lidar_image_delta * 1e9)
            if discard_expired_lidar_head(self.sync_lidar, time.monotonic()):
                # Do not publish a scan after a scheduler/DDS pause. A stale
                # synchronized pair would create a delayed odom sample and
                # can make the planner act on an old pose. Drop only this
                # buffered scan; newer scans remain queued for recovery.
                self.last_sync_lidar_stamp = stamp
                self.sync_counts['lidar_expired'] += 1
                self._audit('sync_rejected', stamp, detail='lidar_expired')
                return
            candidates = self.sync_images
            if self.point_time_mode == 'snapshot':
                # Propagate the snapshot forward to an actual camera measurement.
                candidates = [(t, msg) for t, msg in candidates if t >= stamp]
            image = min(candidates, key=lambda item: abs(item[0] - stamp)) if candidates else None
            coverage_end = max(scan_end, image[0]) if image and self.point_time_mode == 'snapshot' else scan_end
            image_ready = bool(image) if self.point_time_mode == 'snapshot' else (
                bool(self.sync_images) and self.sync_images[-1][0] >= watermark)
            self._audit('sync_check', stamp, image[0] if image else 0,
                        self.sync_imus[-1][0] if self.sync_imus else 0,
                        'ready' if image_ready and self.sync_imus and self.sync_imus[-1][0] >= coverage_end
                        else ('image' if not image_ready else 'imu'))
            if (not image_ready or not self.sync_imus or
                    self.sync_imus[-1][0] < coverage_end):
                return

            delta_ns = abs(image[0] - stamp) if image else None
            start = self.last_sync_lidar_stamp if self.last_sync_lidar_stamp is not None else stamp
            imu_window = [(t, msg) for t, msg in self.sync_imus if start <= t <= coverage_end]
            nearest_imu = min((abs(t - stamp) for t, _ in self.sync_imus), default=None)
            reason = None
            if image is None:
                reason = 'missing_image'
            elif delta_ns > int(self.max_lidar_image_delta * 1e9):
                reason = 'image_delta_exceeded'
            elif (not imu_window or nearest_imu is None or
                  nearest_imu > int(self.max_lidar_imu_delta * 1e9) or
                  imu_window[-1][0] < coverage_end - int(self.max_lidar_imu_delta * 1e9) or
                  (self.point_time_mode == 'snapshot' and
                   (not self.sync_imus or self.sync_imus[-1][0] < coverage_end))):
                reason = 'missing_imu'
            self.sync_lidar.popleft()
            self.last_sync_lidar_stamp = stamp
            if reason:
                self.sync_counts[reason] += 1
                self._audit('sync_rejected', stamp, image[0] if image else 0, detail=reason)
                return

            self.sync_counts['paired'] += 1
            self.sync_deltas_ms.append(delta_ns / 1e6)
            self.sync_wait_ms.append((time.monotonic() - received) * 1000)
        self._audit('publish_lidar_start', stamp, image[0])
        self.pub_sync_lidar.publish(cloud)
        self._audit('publish_image_start', stamp, image[0])
        self.pub_sync_image.publish(image[1])
        self._audit('publish_done', stamp, image[0])

    def _nearest_delta(self, name, stamp):
        with self.history_lock:
            candidates = tuple(self.histories[name])
        if not candidates:
            return None
        return min(abs(stamp - candidate) for candidate in candidates) / 1e9

    def _rate(self, name):
        with self.history_lock:
            values = tuple(self.histories[name])
        if len(values) < 2:
            return 0.0
        elapsed = (values[-1] - values[0]) / 1e9
        return (len(values) - 1) / elapsed if elapsed > 0 else 0.0

    def publish_diagnostics(self):
        if self.stopping:
            return
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'astra_fast_livo2/input_adapter'
        status.hardware_id = 'gazebo_harmonic'
        status.level = DiagnosticStatus.OK
        status.message = 'Input adapter active; estimator status is reported separately'
        for name in ('lidar', 'image', 'imu', 'camera_info'):
            with self.history_lock:
                count = self.counts[name]
            status.values.append(KeyValue(key=name + '_rate_hz', value='%.3f' % self._rate(name)))
            status.values.append(KeyValue(key=name + '_count', value=str(count)))
        with self.sync_lock:
            status.values.append(KeyValue(key='point_time_mode', value=self.point_time_mode))
            status.values.append(KeyValue(key='scan_duration_sec', value=str(self.scan_duration_sec)))
            for name in ('total_lidar', 'paired', 'image_delta_exceeded', 'missing_image',
                         'missing_imu', 'lidar_expired', 'queue_overflow'):
                status.values.append(KeyValue(key='sync_' + name, value=str(self.sync_counts[name])))
            if self.sync_deltas_ms:
                values = sorted(self.sync_deltas_ms)
                status.values.append(KeyValue(key='sync_lidar_rgb_mean_ms', value='%.3f' %
                                              (sum(values) / len(values))))
                status.values.append(KeyValue(key='sync_lidar_rgb_max_ms', value='%.3f' % values[-1]))
                status.values.append(KeyValue(key='sync_lidar_rgb_p95_ms', value='%.3f' %
                                              values[int(0.95 * (len(values) - 1))]))
            if self.sync_wait_ms:
                status.values.append(KeyValue(key='sync_wait_wall_p95_ms', value='%.3f' %
                                              np.percentile(self.sync_wait_ms, 95)))
        with self.history_lock:
            lidar_stamp = self.last_stamp.get('lidar')
        if lidar_stamp is not None:
            image_delta = self._nearest_delta('image', lidar_stamp)
            imu_delta = self._nearest_delta('imu', lidar_stamp)
            status.values.append(KeyValue(key='nearest_lidar_image_delta_s', value=str(image_delta)))
            status.values.append(KeyValue(key='nearest_lidar_imu_delta_s', value=str(imu_delta)))
            if image_delta is None or imu_delta is None:
                status.level = DiagnosticStatus.WARN
                status.message = 'Waiting for synchronized RGB and IMU samples'
            elif image_delta > self.max_lidar_image_delta or imu_delta > self.max_lidar_imu_delta:
                status.level = DiagnosticStatus.WARN
                status.message = 'Input timestamps exceed configured synchronization limits'
        array.status.append(status)
        self.pub_diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    node = FastLivo2Adapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.2)
    finally:
        node.stopping = True
        deadline = time.monotonic() + 4.0
        # Jazzy's Executor.shutdown destroys its guard before worker tasks finish.
        while any(not future.done() for future in executor._futures):
            if time.monotonic() >= deadline:
                node.get_logger().error('Timed out waiting for adapter callbacks to finish')
                break
            time.sleep(0.01)
        for future in tuple(executor._futures):
            if future.done():
                try:
                    future.result()
                except Exception as exc:
                    node.get_logger().error('Callback failed during shutdown:\n' +
                                            ''.join(traceback.format_exception(exc)))
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
