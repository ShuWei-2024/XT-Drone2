#!/usr/bin/env python3
"""Bounded display-only coloured map and optional scan illustration."""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker


class DisplayMap(Node):
    def __init__(self, mesh, stop_file, output, background_alpha=.45):
        super().__init__('astra_rviz_scene', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.mesh,self.stop_file,self.output=mesh,stop_file,output
        self.background_alpha=background_alpha
        self.voxels=OrderedDict(); self.latest=None; self.offset=None
        self.maximum=0;self.frames=0;self.mesh_published=False
        self.last_input=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(PointCloud2,'/astra/ui/color_map',qos)
        self.mesh_pub=self.create_publisher(Marker,'/astra/ui/room_background',qos)
        self.create_subscription(PointCloud2,'/cloud_registered',self.cloud,qos_profile_sensor_data)
        self.create_subscription(String,'/astra/ui/status',self.status,qos)
        self.create_timer(1.,self.publish)

    def cloud(self,msg):
        # Retain only the latest message, with a shallow DDS queue.
        self.latest=msg

    def status(self,msg):
        try:
            d=json.loads(msg.data)
            if self.offset is None and d.get('visual_offset') is not None:
                self.offset=d['visual_offset']
        except (ValueError,TypeError):pass

    def publish(self):
        now=time.monotonic()
        msg,self.latest=self.latest,None
        if msg is not None and msg.header.frame_id=='camera_init':
            names={f.name for f in msg.fields}
            if {'x','y','z','rgb'}<=names:
                points=point_cloud2.read_points(msg,field_names=('x','y','z','rgb'),skip_nans=True).reshape(-1)
                xyz=np.column_stack([points[k] for k in ('x','y','z')])
                rgb=points['rgb'].view(np.uint32) if points['rgb'].dtype.kind=='f' else points['rgb'].astype(np.uint32)
                valid=np.isfinite(xyz).all(axis=1)
                xyz,rgb=xyz[valid],rgb[valid]
                cells=np.floor(xyz/.10).astype(np.int64)
                for cell,p,c in zip(cells,xyz,rgb):
                    key=tuple(cell)
                    self.voxels[key]=(float(p[0]),float(p[1]),float(p[2]),int(c),now)
                    self.voxels.move_to_end(key)
                self.frames+=1
        while self.voxels:
            first=next(iter(self.voxels))
            if len(self.voxels)<=60000 and now-self.voxels[first][4]<=180.:
                break
            self.voxels.popitem(last=False)
        self.maximum=max(self.maximum,len(self.voxels))
        header=Header(frame_id='camera_init',stamp=self.get_clock().now().to_msg())
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1)
                for i,n in enumerate(('x','y','z','rgb'))]
        data=np.empty(len(self.voxels),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        for i,p in enumerate(self.voxels.values()):data[i]=p[:4]
        self.map_pub.publish(point_cloud2.create_cloud(header,fields,data))
        if self.offset is not None and not self.mesh_published:
            m=Marker();m.header=header;m.ns='display_only_room';m.id=0;m.type=Marker.MESH_RESOURCE
            m.mesh_resource=self.mesh.resolve().as_uri();m.mesh_use_embedded_materials=True
            m.pose.orientation.w=1.
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,self.offset)
            m.scale.x=m.scale.y=m.scale.z=1.
            m.color.r=m.color.g=m.color.b=1.;m.color.a=self.background_alpha
            self.mesh_pub.publish(m);self.mesh_published=True
        if self.stop_file.exists():
            self.output.write_text(json.dumps(dict(colour_frames=self.frames,maximum_voxels=self.maximum,
                final_voxels=len(self.voxels),voxel_size_m=.10,capacity=60000,ttl_seconds=180,
                background_offset_display_only=self.offset,planner_input=False),indent=2)+'\n')
            rclpy.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--mesh',type=Path,required=True)
    p.add_argument('--stop-file',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--opaque-background',action='store_true',help='Show outdoor terrain in full colour')
    a=p.parse_args();rclpy.init();node=DisplayMap(a.mesh,a.stop_file,a.output,1. if a.opaque_background else .45)
    try:rclpy.spin(node)
    except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):pass
    finally:
        node.output.write_text(json.dumps(dict(colour_frames=node.frames, maximum_voxels=node.maximum,
            final_voxels=len(node.voxels), voxel_size_m=.10, capacity=60000, ttl_seconds=180,
            background_offset_display_only=node.offset,planner_input=False),indent=2)+'\n')
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
