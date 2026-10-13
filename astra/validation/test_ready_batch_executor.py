"""Exercise idle, ready work, bounds and errors without any flight output."""
import sys
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from ready_batch_executor import ReadyBatchExecutor, BoundedEventsExecutor, receive_ready_batch


@pytest.fixture(params=[ReadyBatchExecutor, BoundedEventsExecutor])
def executor(request):
    rclpy.init()
    executor = request.param()
    yield executor
    executor.shutdown()
    rclpy.shutdown()


def test_idle_stops_at_first_wait(executor):
    # Drain rclpy's initial guard notifications, which need no callbacks.
    result = receive_ready_batch(executor)
    if isinstance(executor, BoundedEventsExecutor):
        assert result['spin_calls'] == 33
        assert result['ready_handlers'] is None
        return
    assert result['spin_calls'] == 1
    assert result['ready_handlers'] == 0
    assert result['idle_wait']


def test_callbacks_from_all_nodes_are_drained(executor):
    nodes = [Node('ready_test_' + str(i)) for i in range(3)]
    received = []
    import threading
    owner = threading.get_ident()
    callback_threads = []
    def callback(msg, i):
        received.append((i, msg.data))
        callback_threads.append(threading.get_ident())
    try:
        for i, node in enumerate(nodes):
            executor.add_node(node)
            node.create_subscription(String, 'ready_batch',
                lambda msg, i=i: callback(msg, i), 10)
        publisher = nodes[0].create_publisher(String, 'ready_batch', 10)
        deadline = time.monotonic() + 3.
        while {i for i, _ in received} != {0, 1, 2} and time.monotonic() < deadline:
            publisher.publish(String(data='unchanged'))
            receive_ready_batch(executor)
        assert {i for i, _ in received} == {0, 1, 2}
        assert all(data == 'unchanged' for _, data in received)
        assert set(callback_threads) == {owner}
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()


def test_original_callback_exception_propagates(executor):
    node = Node('ready_exception_test')
    def fail():
        raise ValueError('original callback failure')
    node.create_timer(.001, fail)
    executor.add_node(node)
    try:
        with pytest.raises(ValueError, match='original callback failure'):
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline:
                receive_ready_batch(executor)
    finally:
        executor.remove_node(node)
        node.destroy_node()


def test_busy_source_cannot_remove_batch_bound():
    class AlwaysReady:
        calls = 0
        def spin_once_ready(self, timeout_sec):
            self.calls += 1
            return True
    executor = AlwaysReady()
    result = receive_ready_batch(executor, drain_sec=1., max_extra_calls=3)
    assert executor.calls == result['spin_calls'] == 4
    assert not result['idle_wait']
    executor.calls = 0
    result = receive_ready_batch(executor, drain_sec=0.)
    assert executor.calls == 1
    assert result['budget_exhausted']


def test_sim_clock_and_wall_timer_both_progress(executor):
    from rosgraph_msgs.msg import Clock
    from rclpy.parameter import Parameter
    from rclpy.clock import Clock as RclClock, ClockType
    nodes = [Node('clock_test_' + str(i),
             parameter_overrides=[Parameter('use_sim_time', value=True)]) for i in range(3)]
    ticks = []
    nodes[0].create_timer(.005, lambda: ticks.append(time.monotonic()),
                          clock=RclClock(clock_type=ClockType.STEADY_TIME))
    publisher = nodes[0].create_publisher(Clock, '/clock', 10)
    msg = Clock(); msg.clock.sec = 42; msg.clock.nanosec = 123000000
    try:
        for node in nodes:executor.add_node(node)
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            publisher.publish(msg)
            receive_ready_batch(executor)
            if ticks and all(n.get_clock().now().nanoseconds == 42123000000 for n in nodes):break
        assert ticks
        assert all(n.get_clock().now().nanoseconds == 42123000000 for n in nodes)
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
