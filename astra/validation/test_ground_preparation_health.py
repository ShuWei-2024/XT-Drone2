"""Disarmed preparation must regain fresh state before the first control send."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))


def test_preparation_backlog_and_later_gap_restart_continuous_health(monkeypatch):
    import ego_sitl_flight as mod
    clock=[0.]
    def spin(*args,**kwargs):clock[0]+=.1
    monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(mod.rclpy,'spin_once',spin)
    def fault():
        return 'odom stale' if clock[0]<.5 or 1.1<=clock[0]<1.3 else ''
    node=NS(fault=fault,initial_alignment_fault=lambda:'')
    mod.await_ground_health(node,required_sec=1.)
    assert clock[0]>=2.3


def test_stale_preparation_times_out_without_sending_commands(monkeypatch):
    import ego_sitl_flight as mod
    clock=[0.]
    def spin(*args,**kwargs):clock[0]+=.1
    monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(mod.rclpy,'spin_once',spin)
    node=NS(fault=lambda:'odom stale',initial_alignment_fault=lambda:'')
    with pytest.raises(RuntimeError,match='preflight not healthy: odom stale'):
        mod.await_ground_health(node,required_sec=1.)
