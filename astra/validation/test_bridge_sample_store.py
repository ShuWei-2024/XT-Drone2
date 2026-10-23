import json
import sys
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from bridge_sample_store import BridgeSampleStore, finish_bridge_samples


def test_bridge_sample_store_spools_full_rows_and_keeps_bounded_tail(tmp_path):
    path = tmp_path / 'bridge_samples.jsonl'
    store = BridgeSampleStore(path)
    rows = [
        {'truth': [0.0, 0.0, 1.0], 'state': 'TRACK_SITL'},
        {'truth': [0.3, 0.4, 1.0], 'state': 'TRACK_SITL'},
        {'truth': [0.6, 0.4, 1.0], 'state': 'TRACK_SITL'},
    ]
    for row in rows:
        store.append(row)

    assert store.count == len(rows)
    assert store.path_length_m == 0.8
    assert len(store.tail) == 2
    assert list(store.tail) == rows[-2:]

    # A live run leaves a complete readable prefix even before close/disarm.
    assert [json.loads(line) for line in path.read_text().splitlines()] == rows

    loaded = store.load()
    assert loaded == rows
    assert len(path.read_text().splitlines()) == len(rows)
    assert all(json.loads(line) == row
               for line, row in zip(path.read_text().splitlines(), rows))


def test_bridge_sample_store_closes_idempotently_and_loads_empty_file(tmp_path):
    store = BridgeSampleStore(tmp_path / 'empty.jsonl')
    store.close()
    store.close()
    assert store.load() == []


def test_many_nested_samples_keep_only_two_in_the_live_store(tmp_path):
    store = BridgeSampleStore(tmp_path / 'many.jsonl')
    for i in range(2000):
        store.append({'truth': [i * .5, 0., 1.], 'diagnostic': {'values': [i] * 32}})
    assert store.count == 2000
    assert store.path_length_m == 999.5
    assert [row['diagnostic']['values'][0] for row in store.tail] == [1998, 1999]
    store.close()
    assert len(store.load()) == 2000


@pytest.mark.parametrize('disarmed', [False, None])
def test_unconfirmed_disarm_keeps_journal_without_loading_full_array(tmp_path, monkeypatch, disarmed):
    store = BridgeSampleStore(tmp_path / 'armed.jsonl')
    store.append({'truth': [0., 0., 1.]})
    monkeypatch.setattr(BridgeSampleStore, 'load', lambda _: pytest.fail('Loaded before disarm'))
    result = {'disarmed': disarmed, '_bridge_sample_store': store}
    finish_bridge_samples(result)
    assert store.stream.closed
    assert result['bridge_samples_file'] == 'armed.jsonl'
    assert result['bridge_sample_count'] == 1
    assert 'bridge_samples' not in result
    assert '_bridge_sample_store' not in result


def test_confirmed_disarm_restores_legacy_audit_array(tmp_path):
    store = BridgeSampleStore(tmp_path / 'done.jsonl')
    row = {'truth': [1., 2., 3.], 'target': None}
    store.append(row)
    result = {'disarmed': True, '_bridge_sample_store': store}
    finish_bridge_samples(result)
    assert result['bridge_samples'] == [row]
    assert result['bridge_sample_count'] == 1
    assert store.stream.closed
