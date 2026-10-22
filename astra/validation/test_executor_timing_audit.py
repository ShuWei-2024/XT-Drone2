"""Profiling must preserve callbacks and their errors, without flight output."""
import sys
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from executor_timing_audit import CompactOdomHistory, ExecutorTimingAudit
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


def test_install_after_ui_creation_traces_new_entities_only_once(node):
    seen=[]
    sub=node.create_subscription(String,'incremental_test',lambda msg:seen.append(msg.data),10)
    original=sub.callback
    audit=ExecutorTimingAudit()
    audit.install(node,with_metadata=False)
    wrapped=sub.callback
    timer=node.create_timer(1.,lambda:seen.append('timer'))
    original_timer=timer.callback
    try:
        audit.install(node,with_metadata=False)
        assert sub.callback is wrapped
        sub.callback(String(data='once'))
        timer.callback()
        assert seen==['once','timer']
        assert audit.stats['executor_audit_test:/incremental_test']['count']==1
    finally:audit.restore()
    assert sub.callback is original and timer.callback is original_timer


def test_gc_pauses_are_recorded_without_changing_collector(node,tmp_path):
    import gc,json
    original=list(gc.callbacks);enabled=gc.isenabled();thresholds=gc.get_threshold()
    audit=ExecutorTimingAudit();audit.install(node)
    try:
        cycle=[];cycle.append(cycle);del cycle
        gc.collect(2)
        row=next(r for r in reversed(audit.gc_events) if r['generation']==2)
        assert row['collected']>=1 and row['uncollectable']==0
        assert row['wall_end_ns']>=row['wall_start_ns']
        assert row['wall_duration_ns']>=row['cpu_duration_ns']>=0
        assert gc.isenabled()==enabled and gc.get_threshold()==thresholds
    finally:audit.restore()
    assert gc.callbacks==original
    audit.save(tmp_path/'trace.json')
    saved=json.loads((tmp_path/'trace.json').read_text())
    assert saved['gc_statistics']['2']['count']>=1


def test_repeated_safety_spans_do_not_accumulate_cyclic_types():
    import gc
    enabled=gc.isenabled()
    gc.collect(2)
    gc.disable()
    try:
        audit=ExecutorTimingAudit()
        for _ in range(2000):
            with audit.span('safety_check'):
                pass
        assert audit.stats['safety_check']['count']==2000
        # The former local class created thousands of unreachable type/MRO/
        # closure objects here, requiring a full collection during flight.
        assert gc.collect(2)<100
    finally:
        if enabled:gc.enable()


def test_compact_history_preserves_full_nanosecond_rows_and_chronological_tail(tmp_path):
    import json
    history=CompactOdomHistory(capacity=3)
    expected=[]
    allocated=len(history.storage)
    for index in range(8):
        row=dict(label=f'node{index%2}:/fast_livo2/odom',topic='/fast_livo2/odom',
                 dds_metadata_available=bool(index%2))
        row.update((key,1_792_394_094_739_581_384+index) for key in history.fields)
        row['measurement_ns']=None if index%2 else 99_123_456_789
        history.append(row);expected.append(row)
        # Changing the transient callback dictionary must not change history.
        row_copy=dict(row);row['wall_start_ns']=0;expected[-1]=row_copy
        assert len(history.storage)==allocated
    assert history.count==8 and len(history)==3
    assert list(history)==expected[-3:]
    assert json.loads(json.dumps(list(history)))==expected[-3:]


def test_audit_export_keeps_odom_schema_and_reports_retention(tmp_path):
    import json
    audit=ExecutorTimingAudit()
    row=dict(label='flight:/fast_livo2/odom',topic='/fast_livo2/odom',
             dds_metadata_available=False)
    row.update((key,10) for key in CompactOdomHistory.fields)
    audit.record(row)
    audit.save(tmp_path/'trace.json')
    saved=json.loads((tmp_path/'trace.json').read_text())
    assert saved['odom_callbacks']==[row]
    assert saved['callback_statistics'][row['label']]['count']==1
    assert saved['odom_history']['total_count']==1
    assert saved['odom_history']['capacity']==65536
    assert saved['odom_history']['overwritten_count']==0
