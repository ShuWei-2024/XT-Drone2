"""Fault acceptance must use actual onboard changes and loss recovery evidence."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from sitl_estimator_reset import reset_parameters, input_ages_valid
import sitl_extended_faults as extended
from sitl_extended_faults import (evaluate_records, latest_live_sample, fault_anchor,
                                 latest_live_clock_anchor)


def fixture(kind):
    injection = dict(command_wall_ns=1_000_000_000, command_sim_us=10_000_000,
        axis='xy', command=43003, requested_counter_changed=True, agent_exit_code=0,
        acknowledgements=[dict(command=43003, result=0)])
    motion = dict(flight_complete=False, disarmed=False, fault='PX4 estimator reset',
        execution_fault_detection=dict(wall_ns=1_200_000_000), bridge_samples=[], events=[
            dict(event='test_aborted', wall_ns=1_210_000_000),
            dict(event='fault_latched', wall_ns=1_220_000_000)])
    data = dict(vehicle_status=dict(timestamp=np.array([10_010_000, 11_500_000, 14_000_000]),
                arming_state=np.array([2, 2, 1]), nav_state=np.array([14, 18, 18])),
        vehicle_land_detected=dict(timestamp=np.array([9_990_000, 12_000_000]), landed=np.array([False, True])),
        vehicle_local_position=dict(timestamp=np.array([9_990_000, 10_010_000]), xy_reset_counter=np.array([1,2])),
        failsafe_flags=dict(timestamp=np.array([11_100_000]), offboard_control_signal_lost=np.array([True])),
        offboard_control_mode=dict(timestamp=np.array([9_990_000])),
        vehicle_command=dict(timestamp=np.array([9_000_000]), command=np.array([176])))
    return injection, motion, data


@pytest.mark.parametrize('kind', ['estimator_reset', 'dds_agent'])
def test_onboard_landing_and_disarm_required_even_if_ros_claims_success(kind):
    injection, motion, data = fixture(kind)
    assert evaluate_records(kind, injection, motion, data, {}, [])['expected_fault_observed']
    motion['disarmed'] = True
    data['vehicle_status']['arming_state'][:] = 2
    assert not evaluate_records(kind, injection, motion, data, {}, [])['expected_fault_observed']


def test_ack_or_forged_counter_is_insufficient_without_onboard_reset():
    injection, motion, data = fixture('estimator_reset')
    data['vehicle_local_position']['xy_reset_counter'][:] = 1
    assert not evaluate_records('estimator_reset', injection, motion, data, {}, [])['expected_fault_observed']


@pytest.mark.parametrize('damage', ['no_loss', 'late_telemetry', 'land_delivered', 'heartbeat_delivered', 'no_land', 'old_curve', 'late_detection'])
def test_link_verdict_rejects_partial_outage_or_missing_recovery(damage):
    injection, motion, data = fixture('dds_agent'); rows = []
    if damage == 'no_loss': data['failsafe_flags']['offboard_control_signal_lost'][:] = False
    if damage == 'late_telemetry': rows=[dict(source='px4', receipt_monotonic_ns=1_600_000_000)]
    if damage == 'land_delivered': data['vehicle_command']=dict(timestamp=np.array([11_000_000]), command=np.array([21]))
    if damage == 'heartbeat_delivered': data['offboard_control_mode']['timestamp']=np.array([11_000_000])
    if damage == 'no_land': data['vehicle_land_detected']['landed'][:]=False
    if damage == 'old_curve': motion['bridge_samples']=[dict(wall_ns=1_300_000_000)]
    if damage == 'late_detection': motion['execution_fault_detection']['wall_ns']=1_600_000_000
    assert not evaluate_records('dds_agent', injection, motion, data, {}, rows)['expected_fault_observed']


def test_xy_reset_does_not_request_altitude_reset_and_yaw_change_is_bounded():
    local=SimpleNamespace(heading=1.); global_position=SimpleNamespace(lat=47.,lon=8.,alt=500.)
    command, params=reset_parameters('xy',local,global_position)
    assert command==43003 and np.isnan(params['param7'])
    assert (params['param5']-47.)*np.pi/180.*6371000. == pytest.approx(.05)
    assert reset_parameters('z',local,global_position)[1]['param7']==500.05
    assert reset_parameters('yaw',local,global_position)[1]['param3']==pytest.approx(np.degrees(1.)+2.)
    with pytest.raises(ValueError):reset_parameters('bad',local,global_position)


def test_live_journal_ignores_partial_final_row_but_rejects_stale_anchor(tmp_path):
    path=tmp_path/'journal.jsonl'
    path.write_text(json.dumps(dict(sim_ns=10,wall_ns=100))+'\n{"sim_ns":')
    assert latest_live_sample(path,200)['sim_ns']==10
    with pytest.raises(ValueError,match='stale'):latest_live_sample(path,300_000_100)


def test_reset_gate_distinguishes_event_status_from_fast_position_stream():
    assert input_ages_valid(dict(local=9.99, global_position=9.98, status=9.5),10.)
    assert not input_ages_valid(dict(local=9.74, global_position=9.98, status=9.5),10.)
    assert not input_ages_valid(dict(local=9.99, global_position=9.98, status=8.99),10.)


def test_missing_injection_input_does_not_interrupt_supervised_flight(tmp_path, monkeypatch):
    handoff=dict(event='ego_control_handoff', wall_ns=1)
    (tmp_path/'motion.log').write_text(json.dumps(handoff)+'\n')
    polls=iter([None,None,0])
    driver=SimpleNamespace(poll=lambda:next(polls),wait=lambda:0)
    agent=SimpleNamespace(pid=123,poll=lambda:None)
    monkeypatch.setattr(extended.os,'getpgid',lambda pid:pid)
    monkeypatch.setattr(extended.time,'monotonic_ns',lambda:2_000_000_000)
    monkeypatch.setattr(extended.time,'sleep',lambda _:None)
    monkeypatch.setattr(extended.os,'killpg',lambda *args:pytest.fail('Must not signal without anchor'))
    assert extended.wait_and_inject(driver,agent,tmp_path,{},tmp_path,'dds_agent','xy',1.,10.)==0
    assert not (tmp_path/'fault_injection.json').exists()
    assert json.loads((tmp_path/'fault_injection_attempt.json').read_text())['injected'] is False


def test_waypoint_fault_waits_for_requested_arrival_not_handoff(tmp_path):
    log=tmp_path/'motion.log'
    events=[dict(event='ego_control_handoff',wall_ns=1),
            dict(event='ego_goal_sent',goal_index=1,wall_ns=2),
            dict(event='ego_waypoint_reached',goal_index=0,wall_ns=3)]
    log.write_text('startup\n'+''.join(json.dumps(e)+'\n' for e in events)+'{partial\n')
    assert fault_anchor(log)['wall_ns']==1
    assert fault_anchor(log,0)['wall_ns']==3
    assert fault_anchor(log,1) is None
    with log.open('a') as stream:
        stream.write(json.dumps(dict(event='ego_waypoint_reached',goal_index=1,wall_ns=4))+'\n')
    assert fault_anchor(log,1)['wall_ns']==4


def test_prepared_reset_is_not_triggered_before_waypoint_arrival(tmp_path, monkeypatch):
    (tmp_path/'motion.log').write_text(json.dumps(dict(event='ego_control_handoff',wall_ns=1))+'\n')
    (tmp_path/'reset_injector_ready').touch()
    polls=iter([None,None,0])
    driver=SimpleNamespace(poll=lambda:next(polls),wait=lambda:0)
    monkeypatch.setattr(extended.time,'sleep',lambda _:None)
    assert extended.wait_and_inject(driver,None,tmp_path,{},tmp_path,
                                   'estimator_reset','xy',1.,10.,waypoint=0)==0
    assert not (tmp_path/'reset_injector_trigger').exists()


def test_hold_clock_uses_live_sent_journal_when_bridge_stopped(tmp_path):
    (tmp_path/'bridge_samples.jsonl').write_text(json.dumps(dict(wall_ns=1,sim_ns=2))+'\n')
    sent=tmp_path/'motion_sent.csv'
    sent.write_text('wall_ns,sim_ns,reason\n300000000,500000000,hold\n400000000,600000000')
    row=latest_live_clock_anchor(tmp_path,400_000_000)
    assert row['wall_ns']==300_000_000
    assert row['clock_anchor_source']=='motion_sent.csv'
    with pytest.raises(ValueError,match='fresh'):latest_live_clock_anchor(tmp_path,600_000_000)


def test_dds_return_requires_actual_landing_and_disarm():
    injection,motion,data=fixture('dds_agent')
    data['vehicle_status']['nav_state'][:]=5
    report=evaluate_records('dds_agent',injection,motion,data,{},[])
    assert report['expected_fault_observed']
    assert report['first_auto_rtl_after_injection_s'] is not None
    data['vehicle_land_detected']['landed'][:]=False
    assert not evaluate_records('dds_agent',injection,motion,data,{},[])['expected_fault_observed']


def test_readonly_observer_waits_for_real_disarm_not_driver_timeout(tmp_path,monkeypatch):
    replies=iter(['arming_state: 2\n','arming_state: 1\n'])
    def query(command,**kwargs):
        assert command==['px4-listener','vehicle_status','-n','1']
        return SimpleNamespace(returncode=0,stdout=next(replies))
    monkeypatch.setattr(extended.subprocess,'run',query)
    monkeypatch.setattr(extended.time,'sleep',lambda _:None)
    assert extended.wait_onboard_disarm(Path('px4-listener'),tmp_path,{},timeout=2.)
    rows=[json.loads(l)for l in (tmp_path/'onboard_recovery_observer.jsonl').read_text().splitlines()]
    assert [r['arming_state']for r in rows]==[2,1]
