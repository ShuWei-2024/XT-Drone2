"""Profiling must preserve callbacks and their errors, without flight output."""
import sys
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from executor_timing_audit import ExecutorTimingAudit
from ready_batch_executor import BoundedEventsExecutor


@pytest.fixture
def node():
    rclpy.init();node=Node('executor_audit_test')
    yield node
    node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('executor_class',[SingleThreadedExecutor,BoundedEventsExecutor])
def test_real_ros_subscription_metadata_and_restore(node,tmp_path,executor_class):
    received=[]
    sub=node.create_subscription(String,'audit_test',lambda msg:received.append(msg.data),10)
    original=sub.callback;audit=ExecutorTimingAudit()
    metadata=executor_class is SingleThreadedExecutor
    audit.install(node,with_metadata=metadata)
    executor=executor_class();executor.add_node(node)
    publisher=node.create_publisher(String,'audit_test',10)
    import time
    deadline=time.monotonic()+3
    while not received and time.monotonic()<deadline:
        publisher.publish(String(data='unchanged'));executor.spin_once(timeout_sec=.05)
    assert received and received[0]=='unchanged'
    record=next(r for r in audit.tail if r.get('topic')=='/audit_test')
    assert record['cpu_duration_ns']>=0 and record['wall_duration_ns']>=0
    assert record['dds_metadata_available'] == metadata
    if metadata:assert record['dds_received_system_ns']>0
    else:assert record['dds_received_system_ns']==0
    audit.restore();assert sub.callback is original
    audit.save(tmp_path/'trace.json');assert (tmp_path/'trace.json').is_file()
    executor.remove_node(node);executor.shutdown()


def test_callback_error_is_preserved_and_recorded(node):
    def fail(msg):raise ValueError('original callback failure')
    sub=node.create_subscription(String,'failure_test',fail,10)
    audit=ExecutorTimingAudit();audit.install(node)
    from types import SimpleNamespace
    with pytest.raises(ValueError,match='original callback failure'):
        sub.callback(String(),SimpleNamespace(received_timestamp=0,source_timestamp=0))
    assert audit.stats['executor_audit_test:/failure_test']['count']==1
    audit.restore();assert sub.callback is fail
