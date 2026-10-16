"""Use the selected serial receiver before arming and after planner detach."""
import gc
import sys
import time
from types import SimpleNamespace as NS
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from ego_sitl_flight import MissionReceiver
from guarded_position_sitl import GuardedPositionTest


@pytest.mark.parametrize('kind', ['ready-batch', 'events'])
def test_preflight_and_recovery_keep_selected_receiver(kind, monkeypatch):
    rclpy.init()
    node = Node('mission_receiver_supervisor')
    extra = Node('mission_receiver_preview')
    callbacks = list(gc.callbacks)
    original = lambda msg: received.append(msg.data)
    received = []
    sub = node.create_subscription(String, '/mission_receiver_test', original, 10)
    pub = extra.create_publisher(String, '/mission_receiver_test', 10)
    receiver = MissionReceiver(node, kind, audit=True)
    node.mission_receiver = receiver
    monkeypatch.setattr(rclpy, 'spin_once', lambda *a, **k: pytest.fail('Old single-callback receiver used'))
    def deliver(value):
        deadline = time.monotonic() + 3.
        while value not in received and time.monotonic() < deadline:
            pub.publish(String(data=value))
            GuardedPositionTest.receive_callbacks(node)
        assert value in received
    try:
        deliver('prestream')
        receiver.attach(extra)
        deliver('execution')
        receiver.executor.remove_node(extra)
        deliver('recovery')
        assert receiver.audit.batch_tail
        assert receiver.audit.stats['mission_receiver_supervisor:/mission_receiver_test']['count'] >= 3
    finally:
        receiver.close()
        assert sub.callback is original
        assert gc.callbacks == callbacks
        extra.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_standalone_guard_preserves_original_receiver(monkeypatch):
    seen = []
    node = object()
    monkeypatch.setattr(rclpy, 'spin_once', lambda n, **kw: seen.append((n, kw)))
    GuardedPositionTest.receive_callbacks(node, timeout_sec=.02)
    assert seen == [(node, {'timeout_sec': .02})]


@pytest.mark.parametrize('reason', [
    'FAST-LIVO2 receipt age exceeds 250 ms',
    'FAST-LIVO2 measurement age outside 0..250 ms',
    'ground-aligned height has no measurement pair within 20 ms'])
@pytest.mark.parametrize('next_reason', ['', 'PX4 estimator reset', 'FAST-LIVO2 receipt age exceeds 250 ms'])
def test_freshness_recheck_is_once_and_all_gates_must_pass(reason, next_reason):
    faults=[reason];calls=[]
    def reconcile():
        calls.append('receive');faults[0]=next_reason
    node=NS(fault=lambda:faults[0],mission_receiver=NS(reconcile=reconcile))
    if next_reason:
        with pytest.raises(RuntimeError,match=next_reason):GuardedPositionTest.check_safe(node)
    else:
        GuardedPositionTest.check_safe(node)
        assert node.queued_state_recoveries==1
    assert calls==['receive'] and node.queued_state_rechecks==1


def test_non_freshness_fault_cannot_be_hidden_by_retry():
    node=NS(fault=lambda:'PX4 estimator reset',
            mission_receiver=NS(reconcile=lambda:pytest.fail('Reset must stop immediately')))
    with pytest.raises(RuntimeError,match='PX4 estimator reset'):GuardedPositionTest.check_safe(node)
