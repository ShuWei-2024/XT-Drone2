"""Transit is not landing permission; recovery never bypasses corridor/health gates."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from flight_recovery import direct_home_mask, compact_mask, landing_allowed
from scan_mesh_guard import MeshGuard


def split_guard(tmp_path):
    meta=dict(minimum_clearance_m=.55,allowed_xy_bounds_m=[[-5.,-5.],[5.,5.]],
              allowed_body_z_bounds_m=[.2,2.],recovery_xy_boxes_m=[[[-1.,-1.],[1.,1.]]],
              recovery_continuous_clearance_lower_bound_m=.65,
              transit_xy_boxes_m=[[[-4.,-4.],[4.,4.]]],transit_body_z_bounds_m=[1.2,2.],
              transit_continuous_clearance_lower_bound_m=.65,
              recovery_policy='planned_return_with_certified_rtl',
              transit_certificate=dict(sampled_bounds_m=[[-5.,-5.],[5.,5.]],certified_z_bounds_m=[1.2,2.]))
    p=tmp_path/'guard.npz'
    np.savez(p,triangles=[[[-10,-10,-5],[10,-10,-5],[0,10,-5]]],metadata=json.dumps(meta))
    return MeshGuard(p)


def test_cruise_can_cross_unlandable_ground_but_not_descend_there(tmp_path):
    g=split_guard(tmp_path)
    assert not g.fault([2.,0.,1.7],transit=True)
    assert 'descent' in g.fault([2.,0.,1.7])
    assert 'height band' in g.fault([2.,0.,1.0],transit=True)
    assert not landing_allowed(g,[2.,0.])
    assert landing_allowed(g,[0.,0.])
    assert not landing_allowed(g,[.8,0.])  # descent needs neighbourhood reserve


def test_direct_rtl_mask_rejects_rectangles_with_an_obstacle_or_boundary():
    free=np.ones((20,20),bool);free[10,13]=False
    mask=direct_home_mask(free,(10,10))
    assert mask[14,10] and not mask[10,16] and not mask[1,10]
    for y,x in np.argwhere(mask):
        assert free[min(y,10)-2:max(y,10)+3,min(x,10)-2:max(x,10)+3].all()
    boxes=compact_mask(mask,[0,0],1.)
    rebuilt=np.zeros_like(mask)
    for (x0,y0),(x1,y1) in boxes:rebuilt[int(y0):int(y1),int(x0):int(x1)]=True
    assert np.array_equal(rebuilt,mask)


def test_new_transit_keeps_existing_landable_flight_area(tmp_path):
    g=split_guard(tmp_path)
    # A landing patch beyond the direct-home transit subset stays available.
    g.recovery_boxes=np.array([[[3.,-1.],[4.8,1.]]])
    g.metadata['transit_includes_descent_region']=True
    assert g.inside_flight_region([4.5,0.]) and landing_allowed(g,[4.5,0.])
    assert not g.inside_flight_region([4.9,0.])


def test_native_rtl_checks_global_home_reset_and_corridor(tmp_path):
    import time
    from guarded_position_sitl import GuardedPositionTest
    keys=('global_position_invalid','home_position_invalid','local_position_invalid',
          'local_altitude_invalid','attitude_invalid','navigator_failure')
    n=NS(mesh_guard=split_guard(tmp_path),fresh=lambda:True,local_wall=time.monotonic(),
         failsafe_flags=NS(**{key:False for key in keys}),flags_wall=time.monotonic(),
         home=NS(valid_hpos=True,valid_lpos=True,valid_alt=True,x=0,y=0,z=0),
         takeoff_origin_ned=[0,0,0,0],resets=(0,0,0),
         local=NS(x=2,y=0,z=-1.5,xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,
                  heading_good_for_control=True),
         recovery_transform=(np.eye(2),np.zeros(2)),ground_route_origin_livo=[0,0],
         planning_envelope=NS(rectangle_free=lambda *a:True),mesh_fault=lambda:'')
    assert not GuardedPositionTest.native_rtl_fault(n)
    n.failsafe_flags.global_position_invalid=True
    assert 'global_position_invalid' in GuardedPositionTest.native_rtl_fault(n)
    n.failsafe_flags.global_position_invalid=False;n.local.xy_reset_counter=1
    assert 'reset' in GuardedPositionTest.native_rtl_fault(n)
    n.local.xy_reset_counter=0;n.home.x=1
    assert 'home differs' in GuardedPositionTest.native_rtl_fault(n)
    n.home.x=0;n.planning_envelope.rectangle_free=lambda *a:False
    assert 'corridor' in GuardedPositionTest.native_rtl_fault(n)
    n.home.x=float('nan')
    assert 'non-finite' in GuardedPositionTest.native_rtl_fault(n)
    n.local.heading_good_for_control=False
    assert 'heading' in GuardedPositionTest.native_rtl_fault(n)


def test_mission_return_locks_queue_and_cancels_pause():
    from rviz_mission import Mission
    m=Mission();m.accepting=True;m.append([2.,0.]);m.command('start',{'end_action':'return_home'})
    m.accepting=True;m.command('pause',{});m.command('return_home',{})
    assert m.return_requested and not m.paused and not m.accepting
    with pytest.raises(ValueError):m.append([3.,0.])


def test_whole_curve_height_checked_even_with_safe_endpoints(tmp_path):
    from scipy.interpolate import BSpline
    from planning_envelope import PlanningEnvelope
    from flight_recovery import spline_height_band
    g=split_guard(tmp_path)
    band=spline_height_band(g,0.)
    env=PlanningEnvelope(g.flight_boxes,g.bounds,[0,0],z_bounds=band)
    knots=[0,0,0,0,1,1,1,1]
    safe=np.array([[0,0,1.5],[.3,0,1.5],[.6,0,1.5],[1,0,1.5]])
    env.validate_spline(BSpline(knots,safe,3))
    # Both endpoints still lie inside the band, but the curve dips below it.
    unsafe=safe.copy();unsafe[1:3,2]=.5
    with pytest.raises(ValueError,match='trajectory leaves'):
        env.validate_spline(BSpline(knots,unsafe,3))


def test_native_rtl_refusal_sends_no_command(monkeypatch):
    from guarded_position_sitl import GuardedPositionTest
    commands=[];events=[]
    n=NS(native_rtl_fault=lambda:'global position invalid',
         command=lambda *a:commands.append(a),event=lambda *a,**k:events.append((a,k)))
    assert not GuardedPositionTest.native_rtl(n)
    assert not commands and events[0][0][0]=='native_rtl_refused'


def test_native_rtl_hands_off_without_offboard_setpoints(monkeypatch):
    import guarded_position_sitl as mod
    from px4_msgs.msg import VehicleStatus as S, VehicleCommand as C
    commands=[];events=[];ticks=[0]
    n=NS(native_rtl_fault=lambda:'',home=NS(x=0,y=0,z=0),truth_body=[0,0,.24],
         status=NS(arming_state=S.ARMING_STATE_ARMED,nav_state=S.NAVIGATION_STATE_OFFBOARD),
         fresh=lambda:True,mesh_fault=lambda:'',event=lambda a,**k:events.append(a),
         command=lambda *a:commands.append(a),send=lambda *a:pytest.fail('Offboard output after handoff'))
    def spin(*a,**k):
        ticks[0]+=1
        if ticks[0]==2:n.status.nav_state=S.NAVIGATION_STATE_AUTO_RTL
        if ticks[0]==3:n.status.arming_state=S.ARMING_STATE_DISARMED
    monkeypatch.setattr(mod.rclpy,'spin_once',spin);monkeypatch.setattr(mod.rclpy,'ok',lambda:True)
    n.receive_callbacks=lambda **kw:mod.GuardedPositionTest.receive_callbacks(n,**kw)
    assert mod.GuardedPositionTest.native_rtl(n)
    assert commands==[(C.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,)]
    assert 'native_rtl_entered' in events and 'native_rtl_disarmed' in events


def test_unsafe_descent_is_refused_when_rtl_unavailable(tmp_path):
    from guarded_position_sitl import GuardedPositionTest
    import time
    events=[]
    n=NS(mesh_guard=split_guard(tmp_path),truth_body=[2,0,1.7],truth_wall=time.monotonic(),
         airborne=True,sample_reason='map stale',native_rtl=lambda:False,
         event=lambda a,**k:events.append(a),command=lambda *a:pytest.fail('Uncertified landing command'))
    assert not GuardedPositionTest.land(n)
    assert events==['landing_refused']


def test_operator_land_at_certified_site_does_not_force_return(tmp_path,monkeypatch):
    import guarded_position_sitl as mod
    import time
    from px4_msgs.msg import VehicleStatus as S, VehicleCommand as C
    commands=[]
    n=NS(mesh_guard=split_guard(tmp_path),truth_body=[0,0,1.7],truth_wall=time.monotonic(),
         airborne=True,sample_reason='operator requested guarded landing',
         native_rtl=lambda:pytest.fail('Operator requested landing at a safe site'),
         event=lambda *a,**k:None,command=lambda *a:commands.append(a),
         status=NS(arming_state=S.ARMING_STATE_ARMED),
         mesh_fault=lambda:'',static_obstacle_fault=lambda:'',
         mesh_guard_xyz=[0,0,1.7],mesh_clearance_m=2.,obstacle_clearance_m=None)
    monkeypatch.setattr(mod.rclpy,'ok',lambda:True)
    monkeypatch.setattr(mod.rclpy,'spin_once',lambda *a,**k:setattr(n.status,'arming_state',S.ARMING_STATE_DISARMED))
    n.receive_callbacks=lambda **kw:mod.GuardedPositionTest.receive_callbacks(n,**kw)
    assert mod.GuardedPositionTest.land(n)
    assert commands==[(C.VEHICLE_CMD_NAV_LAND,)]
