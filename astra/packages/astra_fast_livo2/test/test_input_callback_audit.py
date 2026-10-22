"""Callback exceptions, DDS clocks and GC evidence survive diagnostic tracing."""
import gc
import json
from collections import deque
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from input_callback_audit import (InputCallbackAudit, discard_expired_lidar_head,
                                  stale_lidar_expired)


def test_subscription_preserves_message_and_records_dds_without_restamping():
    rows=[]
    audit=InputCallbackAudit(lambda event,**kwargs:rows.append((event,json.loads(kwargs['detail']))))
    message=object()
    try:
        wrapped=audit.subscription(lambda msg:msg,'image')
        assert wrapped(message,SimpleNamespace(source_timestamp=123,received_timestamp=456)) is message
        dispatch=next(r for event,r in rows if event=='callback_dispatch')
        assert dispatch['dds_source_system_ns']==123
        assert dispatch['dds_received_system_ns']==456
        assert dispatch['callback_system_ns']>456
    finally:audit.close()


def test_failed_callback_still_records_span_and_propagates_exception():
    rows=[]
    audit=InputCallbackAudit(lambda event,**kwargs:rows.append(event))
    def failed():raise ValueError('sensor error')
    try:
        with pytest.raises(ValueError,match='sensor error'):audit.timer(failed,'sync')()
        assert 'callback_span' in rows
    finally:audit.close()


def test_gc_tracing_is_removed_on_shutdown_and_records_interval():
    rows=[]
    audit=InputCallbackAudit(lambda event,**kwargs:rows.append((event,json.loads(kwargs['detail']))))
    try:
        gc.collect(2)
        row=next(r for event,r in rows if event=='adapter_gc' and r['generation']==2)
        assert row['wall_end_ns']>=row['wall_start_ns']
    finally:audit.close()
    assert audit.gc_callback not in gc.callbacks


def test_callback_watchdog_records_python_stall_and_dispatch_stack():
    rows=[]
    audit=InputCallbackAudit(lambda event,**kwargs:rows.append((event,json.loads(kwargs.get('detail','{}')))),
                             stall_threshold_sec=.001)
    try:
        audit.call(lambda: __import__('time').sleep(.02), 'imu', (),
                   SimpleNamespace(source_timestamp=1,received_timestamp=2))
        events={event for event,_ in rows}
        assert 'callback_stall_snapshot' in events
        assert 'callback_queue_stall_snapshot' in events
        queue=next(value for event,value in rows if event=='callback_queue_stall_snapshot')
        assert queue['dds_received_system_ns']==2
        assert queue['threads']
    finally:audit.close()


def test_stale_lidar_is_expired_at_the_existing_three_hundred_ms_gate():
    assert not stale_lidar_expired(10.0, 10.299)
    assert stale_lidar_expired(10.0, 10.300001)


def test_expiration_discards_only_stale_head_and_retains_newer_scans():
    queue=deque([(1, 1.0, 'old'), (2, 10.1, 'fresh')])
    assert discard_expired_lidar_head(queue, 10.301)
    assert list(queue)==[(2,10.1,'fresh')]
    assert not discard_expired_lidar_head(queue, 10.301)
    assert list(queue)==[(2,10.1,'fresh')]


def test_expiration_supports_zero_measurement_stamp():
    queue=deque([(0, 1.0, 'old')])
    assert discard_expired_lidar_head(queue, 1.301)
    assert not queue
