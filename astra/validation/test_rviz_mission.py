"""Mission transition invariants: editing must never restart or retarget a flight."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rviz_mission import Mission


def ready():
    m=Mission();m.accepting=True;return m


def test_clicks_edit_only_and_start_locks_takeoff_without_inventing_return():
    m=ready();m.append([1.,0.]);m.append([2.,0.])
    assert not m.started
    m.command('start', {'end_action':'land','wait_seconds':60})
    assert m.goals==[[1.,0.],[2.,0.]]
    assert not m.accepting
    with pytest.raises(ValueError):m.append([0.,0.])
    with pytest.raises(ValueError):m.command('start',{})


def test_airborne_append_preserves_active_goal_and_order():
    m=ready();m.append([1.,0.]);m.command('start',{'end_action':'hold'})
    m.accepting=True;m.completed=1
    m.append([0.,0.]);m.append([.5,0.])
    assert m.goals==[[1.,0.],[0.,0.],[.5,0.]] and m.completed==1
    with pytest.raises(ValueError):m.command('clear',{})
    with pytest.raises(ValueError):m.command('remove_last',{})
    m.command('pause',{});assert m.paused
    m.command('resume',{});assert not m.paused
    m.close()
    with pytest.raises(ValueError):m.append([2.,0.])
    with pytest.raises(ValueError):m.command('resume',{})


@pytest.mark.parametrize('payload',[{'end_action':'return'},{'wait_seconds':float('nan')},
    {'wait_seconds':0},{'wait_seconds':121},{'wait_seconds':True}])
def test_bad_start_settings_cannot_arm(payload):
    m=ready();m.append([1.,0.])
    with pytest.raises(ValueError):m.command('start',payload)
    assert not m.started


def test_queue_is_bounded_and_rejects_adjacent_duplicates():
    m=ready()
    with pytest.raises(ValueError):m.append([.01,0.])
    for i in range(8):m.append([.3*(i+1),0.])
    with pytest.raises(ValueError):m.append([0.,0.])
    assert len(m.goals)==8


def test_origin_can_be_selected_as_later_waypoint_only():
    from test_rviz_goal_selection import select, guard
    assert select(xyz=[.2,-.1,0.],allow_origin=True,guard=guard([[[-1,-2],[3,1]]]))==pytest.approx([0.,0.])
    with pytest.raises(ValueError):select(xyz=[.2,-.1,0.])


def ground_wait_double(monkeypatch, action_at, cancel=False):
    """Advance wall time beyond five minutes while disarmed and healthy."""
    from types import SimpleNamespace
    import numpy as np
    import rviz_goal_selection as mod
    s=mod.RvizSelection.__new__(mod.RvizSelection)
    s.origin=np.zeros(3);s.height_m=.9;s.stop=False;s.enabled=False;s.mission=Mission()
    s.mission.goals=[[1.,0.]];s.battery_fault=lambda:''
    s.status=lambda:None;s.display=lambda:None
    s.node=SimpleNamespace(phase='',fault=lambda:'',initial_alignment_fault=lambda:'',
        event=lambda *a,**k:None,get_logger=lambda:SimpleNamespace(info=lambda *a:None),
        status=SimpleNamespace(arming_state=1),
        odom=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.,z=0.)))))
    now=[0.]
    def spin(*a,**k):
        now[0]+=1.
        if now[0]>=action_at:
            if cancel:s.stop=True
            else:s.mission.command('start',{})
    monkeypatch.setattr(mod.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(mod.rclpy,'ok',lambda:True)
    s.node.receive_callbacks=spin
    return s,now


def test_ground_editing_can_start_after_old_five_minute_limit(monkeypatch):
    s,now=ground_wait_double(monkeypatch,601.)
    assert s.wait(0.)==[[1.,0.]]
    assert now[0]==601.
    assert s.mission.started


def test_explicit_ground_timeout_still_exits_without_start(monkeypatch):
    s,now=ground_wait_double(monkeypatch,601.)
    with pytest.raises(RuntimeError,match='editing timed out without arming'):s.wait(30.)
    assert now[0]==30.
    assert not s.mission.started


def test_no_ground_timeout_still_respects_operator_close(monkeypatch):
    s,now=ground_wait_double(monkeypatch,301.,cancel=True)
    with pytest.raises(RuntimeError,match='Operator cancelled'):s.wait(0.)
    assert not s.mission.started


def selection_double():
    import json,time
    from types import SimpleNamespace
    from rviz_goal_selection import RvizSelection
    from test_rviz_goal_selection import guard
    s=RvizSelection.__new__(RvizSelection)
    s.session='this-session';s.last_request={};s.ui_wall=time.monotonic()
    s.mission=ready();s.mission.append([1.,0.])
    s.origin=[.2,-.1,-.35];s.height_m=.9
    s.planning_envelope=None
    s.obstacles=[[2.5,-1,.7]];s.obstacle_wall=time.monotonic();s.obstacle_stamp=10_000_000_000
    events=[]
    s.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),
        mesh_guard=guard([[[-1,-2],[3,1]]]),fault=lambda:'',initial_alignment_fault=lambda:'',
        event=lambda name,**kwargs:events.append((name,kwargs)))
    s.status=lambda:None;s.battery_fault=lambda:''
    return s,events


def test_dispatch_rechecks_actual_cruise_height_and_keeps_ground_xy():
    from geometry_msgs.msg import PoseStamped
    s,_=selection_double()
    msg=PoseStamped();msg.header.frame_id='camera_init';msg.header.stamp.sec=10
    msg.pose.position.x=1.4;msg.pose.position.y=-.6
    s.obstacles=[[1.4,-.6,1.0]]
    assert s.check_click(msg)==pytest.approx([1.2,-.5])
    with pytest.raises(ValueError,match='inflated'):s.check_click(msg,target_z=1.0)
    assert s.origin==[.2,-.1,-.35]
    msg.pose.position.z=1.
    with pytest.raises(ValueError,match='2D Goal'):s.check_click(msg,target_z=1.)


def test_stale_wrong_session_and_replayed_commands_never_restart_mission():
    import json
    from std_msgs.msg import String
    s,events=selection_double()
    def send(**overrides):
        data=dict(session=s.session,client='one',request_id=1,stamp_ns=10_000_000_000,
                  command='pause');data.update(overrides)
        s.command(String(data=json.dumps(data)))
    send(session='old',command='land');assert not getattr(s.node,'operator_stop_requested',False)
    send(stamp_ns=1,command='land');assert not getattr(s.node,'operator_stop_requested',False)
    send(command='heartbeat');assert len(s.last_request)==1
    send(command='land');assert not getattr(s.node,'operator_stop_requested',False)
    send(request_id=2,command='land');assert s.node.operator_stop_requested
    assert s.mission.closed and not s.mission.accepting


def test_ui_loss_and_session_deadline_cannot_extend_a_hold():
    import time
    s,_=selection_double();s.mission.started=True;s.flight_deadline=None
    s.ui_wall=time.monotonic()-2.01
    assert s.flight_fault()=='mission control panel heartbeat lost'
    s.ui_wall=time.monotonic();s.flight_deadline=time.monotonic()-1
    assert s.flight_fault()=='mission session duration exceeded'
    s.mission.close();assert not s.flight_fault()
