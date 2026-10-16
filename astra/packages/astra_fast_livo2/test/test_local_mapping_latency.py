"""Verify exact inflation and that map work cannot block safety callbacks."""
import importlib.util
from pathlib import Path
import threading
import time
import unittest

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

path = Path(__file__).resolve().parents[1] / 'scripts' / 'local_mapping_guard.py'
spec = importlib.util.spec_from_file_location('local_mapping_guard', path)
mapping = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapping)


class LocalMapLatencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = mapping.LocalMappingGuard()
        now = self.node.get_clock().now().nanoseconds
        self.node.good_odom = self.node.recovery_count
        self.node.last_odom_ns = self.node.last_cloud_ns = now
        self.node.last_odom_wall = self.node.last_cloud_wall = time.monotonic()
        self.node.last_map_ns = now
        self.node.last_map_wall = time.monotonic()
        self.node.position = self.node.last_odom = np.zeros(3)
        self.node.frame_id = 'camera_init'
        self.node.voxels = {(x, y, 0): now for x in range(5) for y in range(5)}
        self.node.ready = True
        self.release = threading.Event()

    def tearDown(self):
        self.release.set()
        self.node.destroy_node()

    def test_inflation_matches_reference_including_negative_coordinates(self):
        rng = np.random.default_rng(17)
        for count in (1, 50, 500):
            cells = rng.integers(-20, 21, (count, 3), dtype=np.int32)
            expected = np.unique((cells[:, None, :] + self.node.offsets).reshape(-1, 3), axis=0)
            np.testing.assert_array_equal(mapping.inflate_cells(cells, self.node.offsets), expected)
        np.testing.assert_array_equal(mapping.inflate_cells(cells + 1_000_000, self.node.offsets),
                                      expected + 1_000_000)

    def test_sparse_large_extent_uses_exact_union_without_extent_sized_allocation(self):
        cells=np.array([[-10000,-10000,-2],[10000,10000,2]])
        expected=np.unique((cells[:,None,:]+self.node.offsets).reshape(-1,3),axis=0)
        np.testing.assert_array_equal(mapping.inflate_cells(cells,self.node.offsets),expected)

    def test_dense_floor_inflation_is_exact_across_temporary_array_chunks(self):
        cells=np.array([(x,y,-2) for x in range(-40,41) for y in range(-40,41)])
        expected=np.unique((cells[:,None,:]+self.node.offsets).reshape(-1,3),axis=0)
        np.testing.assert_array_equal(mapping.inflate_cells(cells,self.node.offsets),expected)

    def test_worker_does_not_block_odom_or_health_timeout(self):
        build = self.node.build_clouds
        def blocked(*args):
            self.release.wait(timeout=2)
            return build(*args)
        self.node.build_clouds = blocked
        self.node.publish_map()
        first_future = self.node.map_future
        self.node.publish_map()
        self.assertIs(self.node.map_future, first_future)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        calls = []
        def odometry():
            msg = Odometry()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_init'
            self.node.on_odom(msg)
            calls.append(time.monotonic())
        timer = self.node.create_timer(0.02, odometry)
        deadline = time.monotonic() + 0.65
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        self.assertGreaterEqual(len(calls), 20)
        self.assertLess(max(np.diff(calls)), 0.15)
        self.assertFalse(first_future.done())
        self.assertLess(self.node.max_health_interval_ms, 150)
        self.node.last_odom_wall = time.monotonic() - 0.6
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'stale odometry')
        self.node.destroy_timer(timer)
        executor.remove_node(self.node)
        executor.shutdown()

    def test_reset_discards_inflight_map_and_preserves_measurement_stamp(self):
        build = self.node.build_clouds
        def blocked(*args):
            self.release.wait(timeout=2)
            return build(*args)
        self.node.build_clouds = blocked
        stamp = self.node.last_cloud_ns
        self.node.publish_map()
        future = self.node.map_future
        self.node.invalidate('test reset')
        self.release.set()
        occupied, obstacles = future.result(timeout=2)
        self.assertEqual(rclpy.time.Time.from_msg(obstacles.header.stamp).nanoseconds, stamp)
        self.assertGreater(len(point_cloud2.read_points_numpy(obstacles)), len(
            point_cloud2.read_points_numpy(occupied)))
        self.node.finish_map()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.map_results_discarded, 1)
        self.assertFalse(self.node.voxels)

    def test_fresh_inputs_cannot_keep_an_unpublished_map_ready(self):
        self.node.last_map_wall = time.monotonic() - 1.1
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'stale published obstacle map')

    def completed_snapshot(self, expired=False):
        self.node.publish_map()
        self.node.map_future.result(timeout=2)
        self.node.last_map_ns -= 1_100_000_000
        self.node.last_map_wall = time.monotonic() - 1.1
        if expired:
            self.node.map_snapshot_ns -= 1_100_000_000

    def test_fresh_completed_snapshot_replaces_expired_publication_before_health_check(self):
        self.completed_snapshot()
        self.node.check_health()
        self.assertTrue(self.node.ready)
        self.assertIsNone(self.node.map_future)
        self.assertEqual(self.node.map_results_discarded, 0)

    def test_expired_completed_snapshot_cannot_rescue_old_map(self):
        self.completed_snapshot(expired=True)
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'stale published obstacle map')
        self.assertEqual(self.node.map_results_discarded, 1)

    def test_completed_snapshot_cannot_rescue_stale_odometry(self):
        self.completed_snapshot()
        self.node.last_odom_wall = time.monotonic() - .6
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'stale odometry')
        self.assertEqual(self.node.map_results_discarded, 1)

    def test_airframe_filter_keeps_environment_and_original_measurement_time(self):
        self.node.voxels.clear()
        self.node.self_filter_radius = 0.45
        header = Header(frame_id='camera_init')
        header.stamp = rclpy.time.Time(nanoseconds=self.node.last_odom_ns).to_msg()
        cloud = point_cloud2.create_cloud_xyz32(header, np.array([
            [0.2, 0., 0.], [0.6, 0., 0.], [-2., 0., 0.], [6., 0., 0.]
        ], dtype=np.float32))
        self.node.on_cloud(cloud)
        self.assertNotIn((1, 0, 0), self.node.voxels)
        self.assertIn((3, 0, 0), self.node.voxels)
        self.assertIn((-10, 0, 0), self.node.voxels)
        self.assertEqual(len(self.node.voxels), 2)
        self.assertEqual(set(self.node.voxels.values()), {self.node.last_odom_ns})

    def test_point_count_drop_still_invalidates_immediately(self):
        self.node.voxels = dict(list(self.node.voxels.items())[:19])
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'insufficient local obstacle points')

    def test_outdoor_scan_retains_distant_observations_without_inventing_close_points(self):
        self.node.voxels.clear()
        self.node.radius = 12.
        header = Header(frame_id='camera_init')
        header.stamp = rclpy.time.Time(nanoseconds=self.node.last_odom_ns).to_msg()
        xyz = np.array([[6.1, 0, -.3], [10.1, 0, -.3], [13., 0, -.3]], dtype=np.float32)
        self.node.on_cloud(point_cloud2.create_cloud_xyz32(header, xyz))
        self.assertEqual(set(self.node.voxels), {(30, 0, -2), (50, 0, -2)})
        self.assertEqual(set(self.node.voxels.values()), {self.node.last_odom_ns})
        # A wider observation window must not bypass an insufficient map.
        self.node.check_health()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'insufficient local obstacle points')

    def test_larger_window_still_revokes_readiness_at_its_finite_capacity(self):
        self.node.radius = 12.
        self.node.max_voxels = 20000
        now = self.node.get_clock().now().nanoseconds
        # Many unique measured voxels inside the window, exceeding its budget.
        self.node.voxels = {(x, y, z): now for x in range(-20, 21)
                            for y in range(-20, 21) for z in range(13)}
        self.node.publish_map()
        self.assertFalse(self.node.ready)
        self.assertEqual(self.node.reason, 'local map capacity exceeded')
        self.assertFalse(self.node.voxels)
        self.assertIsNone(self.node.map_future)


if __name__ == '__main__':
    unittest.main()
