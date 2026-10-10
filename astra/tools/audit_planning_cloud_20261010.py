"""Record sensor coverage and readiness diagnostics without decoding cloud buffers."""
import argparse
import csv
import json
import time
from pathlib import Path
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--stop-file',required=True);a=p.parse_args()
rclpy.init(); n=Node('planning_cloud_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
rows=[]; stamps=deque(maxlen=100)
def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def cloud(label,m):
    s=stamp(m);rows.append(dict(kind=label,wall_ns=time.monotonic_ns(),stamp_ns=s,count=m.width*m.height,
        paired_odom_ns=min((abs(s-t) for t in stamps),default=-1),frame=m.header.frame_id))
def diag(m):
    for x in m.status:
        rows.append(dict(kind='diagnostics',wall_ns=time.monotonic_ns(),stamp_ns=stamp(m),reason=x.message,
                         **{v.key:v.value for v in x.values}))
n.create_subscription(Odometry,'/fast_livo2/odom',lambda m:stamps.append(stamp(m)),10)
n.create_subscription(PointCloud2,'/fast_livo2/cloud_registered_full',lambda m:cloud('full',m),2)
n.create_subscription(PointCloud2,'/cloud_registered',lambda m:cloud('rgb',m),2)
n.create_subscription(DiagnosticArray,'/local_mapping/diagnostics',diag,10)
end=time.monotonic()+180
try:
    while time.monotonic()<end and not Path(a.stop_file).exists(): rclpy.spin_once(n,timeout_sec=.05)
finally:
    Path(a.output).write_text(json.dumps(rows,indent=2)+'\n');n.destroy_node();rclpy.try_shutdown()
