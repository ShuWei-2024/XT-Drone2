"""The flight GC barrier is limited to its dedicated SITL child process."""

import gc
import subprocess
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from flight_gc_barrier import FlightGcBarrier


def test_opted_in_barrier_collects_and_unfreezes_in_isolated_process():
    # gc.freeze() changes the whole interpreter. Exercise that opt-in only in
    # a child process, so the pytest runner's own GC state is never modified.
    source = f'''
import gc, sys, weakref
sys.path.insert(0, {str(TOOLS)!r})
from flight_gc_barrier import FlightGcBarrier
enabled, thresholds = gc.isenabled(), gc.get_threshold()
cycle = type("Cycle", (), {{}})()
cycle.self = cycle
ref = weakref.ref(cycle)
barrier = FlightGcBarrier(allow_process_wide=True)
barrier.prepare(confirmed_disarmed=True)
assert barrier.freeze_applied and barrier.frozen
assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
del cycle
gc.collect(2)
assert ref() is not None
class YoungCycle: pass
young = YoungCycle(); young.self = young
young_ref = weakref.ref(young)
del young
gc.collect(2)
assert young_ref() is None
barrier.release(confirmed_disarmed=True)
assert barrier.released and not barrier.frozen
assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
assert ref() is None
assert barrier.summary()["thresholds_unchanged"]
'''
    subprocess.run([sys.executable, '-c', source], check=True, capture_output=True, text=True)


def test_default_barrier_preserves_an_existing_freeze_set(monkeypatch):
    calls = []
    monkeypatch.setattr(gc, 'get_freeze_count', lambda: 7)
    monkeypatch.setattr(gc, 'freeze', lambda: calls.append('freeze'))
    monkeypatch.setattr(gc, 'unfreeze', lambda: calls.append('unfreeze'))
    barrier = FlightGcBarrier()
    barrier.prepare(confirmed_disarmed=True)
    assert barrier.prepared and not barrier.freeze_applied and not barrier.frozen
    barrier.release(confirmed_disarmed=True)
    assert barrier.released and calls == []
    assert barrier.summary()['freeze_count_before_prepare'] == 7
    assert barrier.summary()['freeze_count_after_release'] == 7


def test_default_barrier_does_not_take_ownership_of_an_empty_freeze_set(monkeypatch):
    calls = []
    monkeypatch.setattr(gc, 'get_freeze_count', lambda: 0)
    monkeypatch.setattr(gc, 'freeze', lambda: calls.append('freeze'))
    monkeypatch.setattr(gc, 'unfreeze', lambda: calls.append('unfreeze'))
    barrier = FlightGcBarrier()
    barrier.prepare(confirmed_disarmed=True)
    barrier.release(confirmed_disarmed=True)
    assert not barrier.freeze_applied
    assert calls == []


def test_release_is_idempotent():
    barrier = FlightGcBarrier()
    barrier.prepare(confirmed_disarmed=True)
    barrier.release(confirmed_disarmed=True)
    summary = barrier.summary()
    barrier.release(confirmed_disarmed=True)
    assert barrier.summary() == summary


def test_full_collections_require_disarm():
    barrier = FlightGcBarrier()
    with pytest.raises(RuntimeError, match='confirmed disarm'):
        barrier.prepare(confirmed_disarmed=False)
    barrier.prepare(confirmed_disarmed=True)
    barrier.release(confirmed_disarmed=False)
    assert barrier.prepared and not barrier.released
    barrier.release(confirmed_disarmed=True)
    assert barrier.released
