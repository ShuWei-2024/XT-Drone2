"""Exercise the final setpoint handoff with queued measurements and faults."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import ego_sitl_flight as flight


@pytest.fixture
def hold(monkeypatch):
    clock = SimpleNamespace(now=0.)
    node = SimpleNamespace(local=SimpleNamespace(x=99., y=0., z=-1.),
                           events=[], sent=[], checked=0)
    bridge = SimpleNamespace(fault_since=None, fault_reason='', checked=0)
    executor = object()
    def receive(actual_executor):
        assert actual_executor is executor
        clock.now += .01
        # A pending PX4 callback repairs the stale supervisor view.
        node.local.x = 0.
        node.checked += 1
    def check():
        assert node.local.x == 0., 'safety check ran before queued callbacks'
    def health():
        bridge.checked += 1
        return ''
    node.check_safe = check
    node.event = lambda event, **data: node.events.append((event, data))
    node.send = lambda target: node.sent.append((clock.now, target))
    bridge.state_health = health
    monkeypatch.setattr(flight, 'receive_batch', receive)
    monkeypatch.setattr(flight.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(flight.rclpy, 'ok', lambda: True)
    return node, bridge, executor


def test_hold_drains_callbacks_and_has_one_fixed_output(hold):
    node, bridge, executor = hold
    endpoint = (0., 0., -1., 1.57)
    flight.hold_endpoint(node, bridge, executor, endpoint, 3., .3)
    assert node.checked == bridge.checked >= 300
    assert 48 <= len(node.sent) <= 61
    assert all(target == endpoint for _, target in node.sent)
    assert max(b[0]-a[0] for a, b in zip(node.sent, node.sent[1:])) <= .061
    assert [event for event, _ in node.events] == ['segment_start', 'segment_end']


@pytest.mark.parametrize('reason', ['stale local map heartbeat', 'PX4 estimator reset',
                                   'estimator measurement times differ'])
def test_hold_keeps_all_state_faults(hold, reason):
    node, bridge, executor = hold
    bridge.state_health = lambda: reason
    with pytest.raises(RuntimeError, match=reason):
        flight.hold_endpoint(node, bridge, executor, (0., 0., -1., 0.), 3., .3)
    assert not node.sent
    assert not any(event == 'segment_end' for event, _ in node.events)


def test_hold_propagates_supervisor_fault_before_output(hold):
    node, bridge, executor = hold
    def fail():
        raise RuntimeError('source measurement expired')
    node.check_safe = fail
    with pytest.raises(RuntimeError, match='source measurement expired'):
        flight.hold_endpoint(node, bridge, executor, (0., 0., -1., 0.), 3., .3)
    assert not node.sent


def test_hold_rejects_tracking_error(hold):
    node, bridge, executor = hold
    with pytest.raises(RuntimeError, match='tracking error'):
        flight.hold_endpoint(node, bridge, executor, (1., 0., -1., 0.), 3., .3)
    assert not node.sent


@pytest.mark.parametrize('seconds', [0., float('nan'), float('inf'), 301.])
def test_hold_requires_bounded_duration(hold, seconds):
    with pytest.raises(ValueError):
        flight.hold_endpoint(*hold, (0., 0., -1., 0.), seconds, .3)
