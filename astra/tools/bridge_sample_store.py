"""Bounded-memory JSONL spool for high-rate flight audit samples."""

import csv
import json
import math
from collections import deque
from pathlib import Path


class BridgeSampleStore:
    """Persist bridge diagnostics without retaining every nested row in flight.

    These rows are audit evidence, not control state. JSONL is line-buffered so
    an abort still leaves a useful prefix. The complete list is loaded only
    after the vehicle has disarmed; the live process retains just a two-row
    tail plus scalar counters.
    """

    __slots__ = ('path', 'stream', 'count', 'path_length_m', 'previous_xy', 'tail')

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('w', buffering=1)
        self.count = 0
        self.path_length_m = 0.0
        self.previous_xy = None
        self.tail = deque(maxlen=2)

    def append(self, row):
        truth = row.get('truth')
        if truth is not None and len(truth) >= 2:
            xy = (float(truth[0]), float(truth[1]))
            if self.previous_xy is not None:
                self.path_length_m += math.hypot(xy[0]-self.previous_xy[0],
                                                 xy[1]-self.previous_xy[1])
            self.previous_xy = xy
        self.stream.write(json.dumps(row, separators=(',', ':'))+'\n')
        self.count += 1
        self.tail.append(row)

    def close(self):
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()

    def load(self):
        self.close()
        with self.path.open() as stream:
            return [json.loads(line) for line in stream if line.strip()]


def finish_bridge_samples(result):
    """Keep the journal available, but allocate the full list only after disarm."""
    store = result.pop('_bridge_sample_store', None)
    if store is None:
        return
    store.close()
    result['bridge_samples_file'] = str(store.path.name)
    result['bridge_sample_count'] = store.count
    if result.get('disarmed') is True:
        result['bridge_samples'] = store.load()


class SentSampleStore:
    """Write the legacy sent CSV incrementally, retaining no per-send rows."""

    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('w', buffering=1, newline='')
        self.writer = None
        self.count = 0

    def append(self, row):
        if self.writer is None:
            self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.count += 1

    def close(self):
        if not self.stream.closed:
            self.stream.close()


def prepare_flight_logging(node, path):
    """Attach streaming evidence and bound the unused inherited send history."""
    store = SentSampleStore(path)
    node.sent_sample_store = store
    # SettledFlight.send also appends a second history used only by its
    # standalone calibration tool. EGO exports motion_sent.csv instead.
    # Leave standalone calibration behavior intact; this caller owns EGO.
    node.setpoint_rows = deque(node.setpoint_rows, maxlen=2)
    return store
