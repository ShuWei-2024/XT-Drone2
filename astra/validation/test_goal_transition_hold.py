"""Reject direct-next-goal commands while no fresh EGO spline is active."""
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from audit_indoor_hall import transition_hold_audit


def fixture():
    goals=[dict(goal_index=0,wall_ns=0,target_ned=[3,5,-1.7]),
           dict(goal_index=1,wall_ns=100,target_ned=[0,0,-1.7])]
    events=[dict(event='ego_leg_control_handoff',goal_index=1,wall_ns=200)]
    return goals,events


def test_direct_destination_bypass_fails_even_when_in_bounds():
    goals,events=fixture()
    rows=[dict(wall_ns=110,target_x='0',target_y='0',target_z='-1.7')]
    result=transition_hold_audit(goals,events,rows)
    assert not result[0]['passed']
    assert result[0]['maximum_offset_from_completed_endpoint_m']>5


def test_completed_endpoint_is_held_until_handoff():
    goals,events=fixture()
    rows=[dict(wall_ns=110,target_x='3',target_y='5',target_z='-1.7'),
          dict(wall_ns=210,target_x='0',target_y='0',target_z='-1.7')]
    result=transition_hold_audit(goals,events,rows)
    assert result[0]['passed'] and result[0]['native_send_count']==1


def test_aborted_wait_still_checks_for_bypass():
    goals,events=fixture()
    rows=[dict(wall_ns=110,target_x='0',target_y='0',target_z='-1.7')]
    result=transition_hold_audit(goals,[],rows,dict(wall_ns=120))
    assert not result[0]['passed']
