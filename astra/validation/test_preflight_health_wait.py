"""Transient readiness can recover; persistent failures never authorize flight."""
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from health_probe_117 import Probe, wait_for_health


def snapshot(passed):
    return dict(passed=passed, checks={'ready_last_10s':passed},
                ready_last_10s_fraction=1. if passed else .9484536082474226)


def test_recovers_without_restarting_probe_or_lowering_threshold():
    now=[0.]
    samples=iter([snapshot(False),snapshot(True)])
    probe=SimpleNamespace(summary=lambda:next(samples))
    def spin():now[0]+=.5
    result=wait_for_health(probe,12.,60.,spin,monotonic=lambda:now[0],report=lambda *a,**k:None)
    assert result['passed']
    assert result['health_wait']['elapsed_s']==13.
    assert result['health_wait']['attempts'][0]['failed_checks']==['ready_last_10s']
    assert len(result['health_wait']['attempts'])==2


def test_persistent_failure_expires_without_a_pass():
    now=[0.]
    def spin():now[0]+=.5
    result=wait_for_health(SimpleNamespace(summary=lambda:snapshot(False)),12.,15.,spin,
                          monotonic=lambda:now[0],report=lambda *a,**k:None)
    assert not result['passed']
    assert result['health_wait']['elapsed_s']==15.


def test_ready_fraction_cannot_mask_current_false(monkeypatch):
    import health_probe_117
    monkeypatch.setattr(health_probe_117.time,'monotonic',lambda:20.)
    probe=Probe.__new__(Probe)
    probe.rows=defaultdict(list,{key:[19.+i*.05 for i in range(20)] for key in
        ('lidar','camera','imu','odom','ready','px4_local','px4_status','gps','battery','px4_imu')})
    probe.ready=[(19.+i*.05,True) for i in range(19)]+[(19.95,False)]
    probe.odom_ages=[48.];probe.px4_ages=[28.]
    probe.px4_timing=[(1_008_000_000,1_000_000_000,1_004_000_000)]
    probe.truth_times=[1_000_000_000];probe.truth_lock=threading.Lock()
    probe.local=SimpleNamespace(xy_valid=True,z_valid=True)
    result=probe.summary()
    assert result['checks']['ready_last_10s']
    assert not result['checks']['ready_currently_true']
    assert not result['passed']
