"""Winding recovery corridors cannot rely on a straight home path or stale odom."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import time
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from planning_envelope import PlanningEnvelope
from connected_recovery import ConnectedRecovery, recovery_state_fault


def winding_envelope(origin=(0.,0.)):
    # U corridor around a blocked centre: the distant arm has no straight RTL.
    boxes=np.array([[[-3.,-3.],[3.,-1.]],
                    [[-3.,-3.],[-1.,3.]],[[1.,-3.],[3.,3.]]])
    descent=np.array([[[-3.,1.],[-1.,3.]]])
    return PlanningEnvelope(boxes,[[-4.,-4.],[4.,4.]],origin), descent+origin


@pytest.mark.parametrize('origin',[(0.,0.),(.013,-.027)])
def test_every_free_cell_has_a_landing_path_without_crossing_holes(origin):
    env,descent=winding_envelope(origin)
    routes=ConnectedRecovery(env,descent)
    assert np.all(routes.parent[env.free]>=0)
    route=routes.route(np.array([2.,2.])+origin)
    assert len(route)>3
    assert route[-1,0] < -1.+origin[0] and route[-1,1] > 1.+origin[1]
    for a,b in zip(route[:-1],route[1:]):
        assert env.rectangle_free(np.minimum(a,b),np.maximum(a,b))
    assert not env.rectangle_free(np.minimum(route[0],route[-1]),np.maximum(route[0],route[-1]))


def test_disconnected_unrecoverable_component_is_rejected():
    boxes=np.array([[[-3,-3],[-1,3]],[[1,-3],[3,3]]])
    env=PlanningEnvelope(boxes,[[-4,-4],[4,4]],[0,0])
    with pytest.raises(ValueError,match='without a recovery route'):
        ConnectedRecovery(env,boxes[:1])


def test_blocked_start_and_empty_landing_set_are_rejected():
    env,descent=winding_envelope()
    routes=ConnectedRecovery(env,descent)
    with pytest.raises(ValueError,match='outside'):routes.route([0.,0.])
    with pytest.raises(ValueError,match='no certified landing'):
        ConnectedRecovery(env,np.array([[[0.,0.],[.1,.1]]]))


def state():
    from px4_msgs.msg import VehicleStatus
    return NS(fresh=lambda:True,local_wall=time.monotonic(),truth_wall=time.monotonic(),
        local=NS(x=0.,y=0.,z=-1.5,vx=0.,vy=0.,vz=0.,heading=0.,xy_valid=True,z_valid=True,
            v_xy_valid=True,v_z_valid=True,heading_good_for_control=True,
            xy_reset_counter=1,z_reset_counter=2,heading_reset_counter=3),
        resets=(1,2,3),status=NS(nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD),
        recovery_transform=(np.eye(2),np.zeros(2)),recovery_z_translation=0.,
        ground_route_origin_livo=[0.,0.],truth_body=[0.,0.,1.74],truth=[0.,0.,1.5],
        vertical_reference=0.,truth_body_velocity=[0.,0.,0.],
        planning_envelope=NS(contains=lambda xy:True,z_bounds=[1.,1.8]),
        mesh_fault=lambda:'',mesh_guard=NS(approaching_boundary=lambda *a,**kw:''))


def test_recovery_does_not_need_livo_but_requires_fresh_valid_px4_and_fixed_epoch():
    n=state()
    assert recovery_state_fault(n)==''  # deliberately has no odom or readiness
    n.local.xy_reset_counter+=1
    assert 'reset' in recovery_state_fault(n)
    n=state();n.local_wall-=.3
    assert 'stale' in recovery_state_fault(n)
    n=state();n.local.v_xy_valid=False
    assert 'invalid' in recovery_state_fault(n)
    n=state();n.local.x=.31
    assert 'XY disagreement' in recovery_state_fault(n)
    n=state();n.local.z=-1.9
    assert 'outside shared' in recovery_state_fault(n)


def test_unlandable_site_uses_connected_route_before_land(monkeypatch):
    import guarded_position_sitl as mod
    import connected_recovery
    from px4_msgs.msg import VehicleStatus,VehicleCommand
    events=[];commands=[]
    n=state();n.truth_body=[2.,0.,1.74];n.airborne=True;n.sample_reason='map stale'
    n.connected_recovery=object()
    n.mesh_guard=NS(transit_boxes=np.ones((1,2,2)),recovery_boxes=np.array([[[-1.,-1.],[1.,1.]]]))
    n.event=lambda name,**kw:events.append(name)
    n.command=lambda *a:commands.append(a)
    n.status.arming_state=VehicleStatus.ARMING_STATE_ARMED
    n.native_rtl=lambda:pytest.fail('Expanded recovery must not assume a straight RTL corridor')
    n.receive_callbacks=lambda **kw:setattr(n.status,'arming_state',VehicleStatus.ARMING_STATE_DISARMED)
    n.mesh_fault=lambda:'';n.static_obstacle_fault=lambda:''
    n.mesh_guard_xyz=[0.,0.,1.74];n.mesh_clearance_m=1.;n.obstacle_clearance_m=None
    def recover(node):
        node.truth_body=[0.,0.,1.74]
        return True
    monkeypatch.setattr(connected_recovery,'execute_connected_recovery',recover)
    monkeypatch.setattr(mod.rclpy,'ok',lambda:True)
    assert mod.GuardedPositionTest.land(n)
    assert commands==[(VehicleCommand.VEHICLE_CMD_NAV_LAND,)]
    assert 'disarmed' in events


def test_failed_connected_recovery_never_commands_descent(monkeypatch):
    import guarded_position_sitl as mod
    import connected_recovery
    n=state();n.truth_body=[2.,0.,1.74];n.airborne=True;n.sample_reason='map stale'
    n.connected_recovery=object()
    n.mesh_guard=NS(transit_boxes=np.ones((1,2,2)),recovery_boxes=np.array([[[-1.,-1.],[1.,1.]]]))
    n.event=lambda *a,**kw:None
    n.command=lambda *a:pytest.fail('No landing over an uncertified site')
    monkeypatch.setattr(connected_recovery,'execute_connected_recovery',lambda _:False)
    assert not mod.GuardedPositionTest.land(n)


def test_operator_land_in_transit_uses_nearby_landing_policy_without_forcing_home():
    from rviz_goal_selection import RvizSelection
    from rviz_mission import Mission
    mission=Mission();mission.accepting=True;mission.append([2.,0.]);mission.command('start',{})
    n=NS(truth_body=[2.,0.,1.74],event=lambda *a,**kw:None,
         mesh_guard=NS(recovery_boxes=np.array([[[-1.,-1.],[1.,1.]]]),
             metadata={'recovery_policy':'connected_route_to_certified_landing'}))
    selection=NS(node=n,mission=mission,stop=False)
    RvizSelection.request_land(selection)
    assert selection.stop and n.operator_stop_requested and mission.closed
    assert not mission.return_requested
