"""A failed mission is only a successful fault test with bounded, safe recovery."""
import copy
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from sitl_fault_regression import (EXPECTED_REASONS, evaluate_fault,
    open_owned_child_pidfd, pidfd_exited)


def case(kind):
    injection=dict(command_wall_ns=1_000_000_000)
    motion=dict(disarmed=True,flight_complete=False,fault=EXPECTED_REASONS[kind][0],
                bridge_samples=[dict(wall_ns=1_200_000_000)],events=[
                    dict(event='test_aborted',wall_ns=1_250_000_000),
                    dict(event='fault_latched',wall_ns=1_251_000_000),
                    dict(event='land_requested',wall_ns=1_260_000_000)])
    if kind!='position_relay':motion['events'].append(dict(event='hold_target_frozen',wall_ns=1_251_000_000))
    return injection,motion


@pytest.mark.parametrize('kind',list(EXPECTED_REASONS))
def test_expected_failure_needs_actual_abort_bounded_reaction_and_disarm(kind):
    injection,motion=case(kind)
    assert evaluate_fault(kind,injection,1,0,motion)['expected_fault_observed']
    for field,value in [('disarmed',False),('flight_complete',True),('fault','unrelated numerical error')]:
        broken=copy.deepcopy(motion);broken[field]=value
        assert not evaluate_fault(kind,injection,1,0,broken)['expected_fault_observed']
    assert not evaluate_fault(kind,None,1,0,motion)['expected_fault_observed']
    # A target-identification failure leaves an empty injection record.  It
    # must be reported as a failed test, never as an observed source outage.
    empty_injection=evaluate_fault(kind,{},1,0,motion)
    assert not empty_injection['expected_fault_observed']
    assert not empty_injection['checks']['injected']
    assert empty_injection['response_after_injection_ms'] is None
    assert not evaluate_fault(kind,injection,0,0,motion)['expected_fault_observed']
    assert not evaluate_fault(kind,injection,1,None,motion)['expected_fault_observed']
    broken=copy.deepcopy(motion)
    for e in broken['events']:
        if e['event'] in ('fault_latched','hold_target_frozen'):e['wall_ns']=1_501_000_000
    assert not evaluate_fault(kind,injection,1,0,broken)['expected_fault_observed']
    broken=copy.deepcopy(motion);broken['bridge_samples'].append(dict(wall_ns=1_255_000_000))
    assert not evaluate_fault(kind,injection,1,0,broken)['expected_fault_observed']


def test_stale_position_is_not_used_to_invent_a_hold_target():
    injection,motion=case('position_relay')
    report=evaluate_fault('position_relay',injection,1,0,motion)
    assert report['expected_fault_observed']
    assert report['hold_after_injection_ms'] is None
    assert report['response_event']=='fault_latched'
    motion['events']=[e for e in motion['events'] if e['event']!='land_requested']
    assert not evaluate_fault('position_relay',injection,1,0,motion)['expected_fault_observed']


@pytest.mark.parametrize('reason',[
    'endpoint hold state fault: stale local map heartbeat',
    'bridge fault: stale local map heartbeat'])
def test_map_heartbeat_failure_is_recognized_in_each_execution_phase(reason):
    injection,motion=case('map');motion['fault']=reason
    assert evaluate_fault('map',injection,1,0,motion)['expected_fault_observed']
    motion['disarmed']=False
    assert not evaluate_fault('map',injection,1,0,motion)['expected_fault_observed']


def test_pidfd_targets_only_the_exact_owned_child_and_keeps_launcher_alive():
    launcher=subprocess.Popen([sys.executable,'-c',
        "import subprocess,time; p=subprocess.Popen(['/bin/sleep','30']); "
        "print(p.pid,flush=True); time.sleep(30)"],
        start_new_session=True,stdout=subprocess.PIPE,text=True)
    unrelated=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)
    descriptor=None
    try:
        expected=int(launcher.stdout.readline().strip())
        child,descriptor=open_owned_child_pidfd(launcher.pid,'/bin/sleep')
        assert child==expected and child!=unrelated.pid
        assert not pidfd_exited(descriptor)
        signal.pidfd_send_signal(descriptor,signal.SIGINT)
        deadline=time.monotonic()+2.
        while not pidfd_exited(descriptor):
            assert time.monotonic()<deadline
            time.sleep(.01)
        assert launcher.poll() is None and unrelated.poll() is None
        with pytest.raises(ValueError,match='found 0'):
            open_owned_child_pidfd(launcher.pid,'/bin/true')
    finally:
        if descriptor is not None:os.close(descriptor)
        os.killpg(launcher.pid,signal.SIGKILL)
        unrelated.kill();unrelated.wait();launcher.wait();launcher.stdout.close()


def test_external_child_exit_needs_observation_without_claiming_an_exit_code():
    injection,motion=case('livo')
    assert not evaluate_fault('livo',injection,1,None,motion)['expected_fault_observed']
    report=evaluate_fault('livo',injection,1,None,motion,observed_child_exit=True)
    assert report['expected_fault_observed']
    assert report['stopped_process_exit_code'] is None
    assert report['child_exit_confirmed_by_pidfd']
