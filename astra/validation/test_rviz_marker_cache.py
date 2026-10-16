"""Unchanged UI geometry must not block the flight receiver every second."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rviz_goal_selection import RvizSelection
from rviz_mission import Mission


def test_region_cache_keeps_full_latched_display_and_updates_waypoints():
    s=RvizSelection.__new__(RvizSelection)
    published=[]
    s.node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=10))),
              mesh_guard=NS(transit_boxes=np.array([[[0,0],[4,4]]]),
                            recovery_boxes=np.array([[[0,0],[1,1]],[[1,1],[2,2]]]),
                            metadata={'recovery_certificate':{'compacted':True}}))
    s.origin=np.array([.1,.2,0.]);s.height_m=1.5;s.mission=Mission()
    s.region_markers=None;s.display_signature=None
    s.pub=NS(publish=published.append)
    s.display()
    regions=s.region_markers
    assert len(regions)==3
    assert len(published)==1 and published[0].markers[0].action==Marker.DELETEALL
    for _ in range(5):s.display()
    assert len(published)==1
    s.mission.goals.append([2.,0.]);s.display()
    assert s.region_markers is regions and len(published)==2
    assert [m.id for m in published[-1].markers]==[0,100000,1000,1001,10,11]
    goal=next(m for m in published[-1].markers if m.id==10)
    assert goal.color.r==1.
    s.mission.completed=1;s.display()
    goal=next(m for m in published[-1].markers if m.id==10)
    assert goal.color.g==1. and goal.color.r<.3
    s.mission.goals.clear();s.display()
    assert [m.id for m in published[-1].markers]==[0,100000,1000,1001]
