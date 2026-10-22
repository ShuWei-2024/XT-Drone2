"""Flight sends retain CSV evidence without keeping every row in memory."""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from bridge_sample_store import SentSampleStore, prepare_flight_logging
from guarded_position_sitl import GuardedPositionTest
from px4_calibration_flight import CalibrationFlight


def sample_node():
    return NS(rows=[], phase='endpoint_hold', odom_age=lambda: .04,
              odom_wall=0., ready=True, deferred_odom_count=0,
              truth=(1., 2., 3.), mesh_clearance_m=.8,
              local=NS(x=1., y=2., z=-3., vx=0., vy=0., vz=0.),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=123)))


def test_sent_csv_preserves_quoting_and_complete_rows_without_retaining_them(tmp_path):
    store = SentSampleStore(tmp_path / 'motion_sent.csv')
    for i in range(2000):
        store.append({'sim_ns': i, 'reason': '保持,\n检查', 'mesh_clearance_m': .8})
    assert store.count == 2000
    assert not hasattr(store, 'rows')
    store.close()
    store.close()
    rows = list(csv.DictReader(store.path.open()))
    assert len(rows) == 2000
    assert rows[-1] == {'sim_ns': '1999', 'reason': '保持,\n检查', 'mesh_clearance_m': '0.8'}


def test_guarded_samples_stream_all_fields_with_no_in_memory_rows(tmp_path):
    node = sample_node()
    node.sent_sample_store = SentSampleStore(tmp_path / 'sent.csv')
    for _ in range(100):
        GuardedPositionTest.sample(node, (1., 2., -3.), 'test')
    assert node.rows == []
    assert node.sent_sample_store.count == 100
    node.sent_sample_store.close()
    rows = list(csv.DictReader((tmp_path / 'sent.csv').open()))
    assert len(rows) == 100
    assert rows[0]['phase'] == 'endpoint_hold'
    assert rows[0]['mesh_clearance_m'] == '0.8'
    assert rows[0]['target_z'] == '-3.0'


def test_standalone_guard_keeps_existing_in_memory_sampling():
    node = sample_node()
    GuardedPositionTest.sample(node, (1., 2., -3.))
    assert len(node.rows) == 1
    assert node.rows[0]['mesh_clearance_m'] == .8


def test_logging_error_aborts_once_but_recovery_samples_remain_possible_and_bounded():
    node = sample_node()
    def fail(_):
        raise OSError('disk full')
    node.sent_sample_store = NS(append=fail)
    with pytest.raises(OSError, match='disk full'):
        GuardedPositionTest.sample(node, (1., 2., -3.))
    for _ in range(100):
        GuardedPositionTest.sample(node, (1., 2., -3.), 'recovery')
    assert node.sent_sample_store is None
    assert len(node.rows) == 2
    assert all(row['reason'] == 'recovery' for row in node.rows)


def test_ego_bounds_inherited_send_history_and_still_sends_every_target(tmp_path, monkeypatch):
    node = GuardedPositionTest.__new__(GuardedPositionTest)
    node.__dict__.update(vars(sample_node()))
    node.sample_reason = ''
    node.local.timestamp_sample = 10
    node.local.timestamp = 11
    node.setpoint_rows = []
    sent = []
    monkeypatch.setattr(CalibrationFlight, 'send', lambda _, target: sent.append(target))
    store = prepare_flight_logging(node, tmp_path / 'sent.csv')
    for i in range(1000):
        target = (float(i), 0., -1., 1.57)
        node.send(target)
    assert len(sent) == store.count == 1000
    assert len(node.setpoint_rows) == 2
    assert node.setpoint_rows[-1]['target_x_ned_m'] == 999.
    assert node.rows == []
    store.close()
