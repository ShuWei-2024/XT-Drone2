"""Check that snapshot timing removes only the artificial scan window."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
import rclpy
from sensor_msgs.msg import Image, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


path = Path(__file__).resolve().parents[1] / 'scripts' / 'astra_fast_livo2_adapter.py'
spec = importlib.util.spec_from_file_location('adapter_snapshot', path)
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class SnapshotTimingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def make_node(self, mode):
        node = adapter.FastLivo2Adapter()
        node.point_time_mode = mode
        node.scan_duration_sec = 0.0 if mode == 'snapshot' else 0.1
        self.addCleanup(node.destroy_node)
        return node

    def inputs(self, node):
        header = Header()
        header.stamp.sec = 10
        cloud = point_cloud2.create_cloud_xyz32(header, np.array([
            [1., 0., 0.], [np.nan, 0., 0.], [2., 0., 0.]], dtype=np.float32))
        node.lidar_callback(cloud)
        image = Image()
        image.header.stamp.sec = 10
        node.image_callback(image)
        watermark = Image()
        watermark.header.stamp.sec = 10
        watermark.header.stamp.nanosec = 32_000_000
        node.image_callback(watermark)
        imu = Imu()
        imu.header.stamp.sec = 10
        node.imu_callback(imu)

    def test_snapshot_offsets_are_zero_and_header_is_preserved(self):
        node = self.make_node('snapshot')
        self.inputs(node)
        cloud = node.sync_lidar[0][2]
        np.testing.assert_array_equal(point_cloud2.read_points(cloud)['time'], [0., 0.])
        self.assertEqual(adapter.stamp_ns(cloud), 10_000_000_000)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 1)

    def test_synthetic_scan_still_waits_for_end_imu_and_keeps_indices(self):
        node = self.make_node('synthetic_scan')
        self.inputs(node)
        cloud = node.sync_lidar[0][2]
        np.testing.assert_array_equal(point_cloud2.read_points(cloud)['time'], [0., 100_000.])
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 0)
        self.assertEqual(len(node.sync_lidar), 1)
        imu = Imu()
        imu.header.stamp.sec = 10
        imu.header.stamp.nanosec = 100_000_000
        node.imu_callback(imu)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 1)

    def test_snapshot_still_rejects_missing_imu(self):
        node = self.make_node('snapshot')
        self.inputs(node)
        node.sync_imus.clear()
        stamp, received, cloud = node.sync_lidar[0]
        node.sync_lidar[0] = (stamp, received - 1., cloud)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 0)
        self.assertEqual(node.sync_counts['missing_imu'], 1)

    def test_snapshot_uses_future_image_without_waiting_for_next_scan(self):
        node = self.make_node('snapshot')
        self.inputs(node)
        node.sync_images.clear()
        node.last_stamp.pop('image', None)
        for ns in (-4_000_000, 12_000_000):
            image = Image()
            image.header.stamp.sec = 10 if ns >= 0 else 9
            image.header.stamp.nanosec = ns if ns >= 0 else 1_000_000_000 + ns
            node.image_callback(image)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 0)
        imu = Imu()
        imu.header.stamp.sec = 10
        imu.header.stamp.nanosec = 16_000_000
        node.imu_callback(imu)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 1)
        self.assertEqual(list(node.sync_deltas_ms), [12.])

    def test_snapshot_does_not_fuse_image_before_cloud(self):
        node = self.make_node('snapshot')
        self.inputs(node)
        node.sync_images.clear()
        node.last_stamp.pop('image', None)
        image = Image()
        image.header.stamp.sec = 9
        image.header.stamp.nanosec = 996_000_000
        node.image_callback(image)
        stamp, received, cloud = node.sync_lidar[0]
        node.sync_lidar[0] = (stamp, received - 1., cloud)
        node.process_sync()
        self.assertEqual(node.sync_counts['paired'], 0)
        self.assertEqual(node.sync_counts['missing_image'], 1)


if __name__ == '__main__':
    unittest.main()
