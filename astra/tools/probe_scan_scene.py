#!/usr/bin/env python3
"""Passive sensor evidence for an imported scan; never publishes flight commands."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class Probe(Node):
    def __init__(self, output, capture=False):
        super().__init__('scan_scene_probe')
        self.output = output
        self.counts = dict(lidar=0, rgb=0, imu=0, odom=0, registered_cloud=0, ready=0)
        self.points = []; self.ready = []; self.poses = []; self.rgb = None
        self.first_lidar = None
        self.capture=capture
        self.imu_rows=[];self.truth_rows=[];self.cloud_samples=[]
        self.gz=None
        if capture:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GazeboNode
            self.gz=GazeboNode()
            self.gz.subscribe(Pose_V,'/world/astra_px4_open/dynamic_pose/info',self.truth)
        self.create_subscription(PointCloud2, '/sensors/lidar/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self.registered, qos_profile_sensor_data)
        self.create_subscription(Image, '/sensors/camera/color/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(Imu, '/sensors/imu/data', self.imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fast_livo2/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/local_mapping/ready', self.readiness,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def bump(self, key): self.counts[key] += 1

    def truth(self,msg):
        pose=next((x for x in msg.pose if x.name=='astra_x500_px4'),None)
        if pose:
            p,q=pose.position,pose.orientation
            self.truth_rows.append([msg.header.stamp.sec*10**9+msg.header.stamp.nsec,
                                   p.x,p.y,p.z,q.x,q.y,q.z,q.w])

    def imu(self,msg):
        self.bump('imu')
        if self.capture:
            p,q=msg.linear_acceleration,msg.angular_velocity
            self.imu_rows.append([msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                                  p.x,p.y,p.z,q.x,q.y,q.z])

    def cloud(self, msg):
        self.bump('lidar')
        fields = {f.name:f for f in msg.fields}
        if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):
            raise ValueError('Expected float32 XYZ cloud')
        dtype = np.dtype(dict(names=['x','y','z'], formats=['<f4']*3,
            offsets=[fields[k].offset for k in ('x','y','z')], itemsize=msg.point_step))
        raw = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step,msg.point_step))
        xyz = np.column_stack([raw[k].ravel() for k in ('x','y','z')])
        ranges = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(xyz).all(axis=1) & (ranges>.2) & (ranges<20.)
        self.points.append(int(valid.sum()))
        if self.capture and self.counts['lidar']%10==1 and valid.sum()>1000:
            sample=xyz[valid][np.linspace(0,valid.sum()-1,256,dtype=int)]
            self.cloud_samples.append((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,sample.copy()))
        if self.first_lidar is None and valid.sum()>1000:
            self.first_lidar=xyz[valid]
            np.save(self.output/'scan_lidar_xyz.npy', self.first_lidar)

    def registered(self, msg): self.bump('registered_cloud')

    def image(self, msg):
        self.bump('rgb')
        if self.rgb is not None: return
        channels = {'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4}.get(msg.encoding)
        if not channels: return
        rgb = np.ndarray((msg.height,msg.width,channels), dtype='u1', buffer=msg.data,
                         strides=(msg.step,channels,1))[:,:,:3].copy()
        if msg.encoding.startswith('bgr'): rgb=rgb[:,:,::-1]
        if self.counts['rgb']==1:
            PILImage.fromarray(rgb).save(self.output/'scan_camera_first.png')
        if rgb.std()<5: return
        self.rgb=rgb
        PILImage.fromarray(rgb).save(self.output/'scan_camera.png')

    def odom(self, msg):
        self.bump('odom');p=msg.pose.pose.position
        self.poses.append([p.x,p.y,p.z])

    def readiness(self, msg):
        self.bump('ready');self.ready.append(msg.data)

    def finish(self, seconds):
        if self.capture:
            self.gz.unsubscribe('/world/astra_px4_open/dynamic_pose/info')
            for name,header,rows in [('raw_imu',['stamp_ns','ax','ay','az','gx','gy','gz'],self.imu_rows),
                                    ('raw_truth',['stamp_ns','x','y','z','qx','qy','qz','qw'],self.truth_rows)]:
                with (self.output/(name+'.csv')).open('w') as stream:
                    writer=csv.writer(stream);writer.writerow(header);writer.writerows(rows)
            if self.cloud_samples:
                np.savez_compressed(self.output/'raw_cloud_samples.npz',
                    stamps=np.array([x[0] for x in self.cloud_samples],dtype=np.int64),
                    xyz=np.stack([x[1] for x in self.cloud_samples]))
        xyz=self.first_lidar
        result=dict(counts=self.counts, duration_wall_s=seconds,
            lidar_finite_point_count_min=min(self.points) if self.points else 0,
            lidar_finite_point_count_median=float(np.median(self.points)) if self.points else 0,
            ready_fraction=sum(self.ready)/len(self.ready) if self.ready else None,
            camera_std=float(self.rgb.std()) if self.rgb is not None else None,
            camera_resolution=list(self.rgb.shape[:2][::-1]) if self.rgb is not None else None,
            lidar_observed_xyz_bounds=[xyz.min(axis=0).tolist(),xyz.max(axis=0).tolist()] if xyz is not None else None,
            static_odom_span_m=np.ptp(self.poses,axis=0).tolist() if self.poses else None,
            publishes_control=False, flight_validated=False)
        result['passed']=bool(self.rgb is not None and len(self.points)>100 and
            min(self.points)>1000 and self.counts['imu']>1000 and self.counts['odom']>100 and
            self.counts['registered_cloud']>100 and any(self.ready))
        (self.output/'scan_probe.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--stop-file',type=Path)
    p.add_argument('--capture-inputs',action='store_true')
    a=p.parse_args();rclpy.init();node=Probe(a.output,a.capture_inputs);start=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start<a.seconds:
            if a.stop_file and a.stop_file.exists():break
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.finish(time.monotonic()-start);node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()
