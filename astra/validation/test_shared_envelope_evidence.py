"""Lost ROS confirmation must not discard independently recorded execution."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from audit_shared_envelope import recorded_bridge_samples


def test_severed_link_loads_closed_journal_without_fabricating_ros_disarm(tmp_path):
    row=dict(state='TRACK_SITL', preview_trajectory_id=7)
    (tmp_path/'bridge_samples.jsonl').write_text(json.dumps(row)+'\n')
    motion=dict(disarmed=False, bridge_sample_count=1)
    assert recorded_bridge_samples(tmp_path,motion)==[row]
    assert motion==dict(disarmed=False,bridge_sample_count=1)


def test_missing_or_truncated_journal_cannot_certify_executed_curves(tmp_path):
    with pytest.raises(FileNotFoundError):
        recorded_bridge_samples(tmp_path,dict(bridge_sample_count=1))
    (tmp_path/'bridge_samples.jsonl').write_text('')
    with pytest.raises(ValueError,match='count mismatch'):
        recorded_bridge_samples(tmp_path,dict(bridge_sample_count=1))


def test_legacy_inline_execution_evidence_remains_readable(tmp_path):
    rows=[dict(state='TRACK_SITL',preview_trajectory_id=8)]
    assert recorded_bridge_samples(tmp_path,dict(bridge_samples=rows))==rows
