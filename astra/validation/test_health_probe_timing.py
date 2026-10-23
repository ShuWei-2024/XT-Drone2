"""The passive monitor distinguishes source cadence from its own callback cadence."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from health_probe_117 import Probe, stamp_ns


def test_channel_summary_separates_delivery_gap_from_measurement_gap():
    rows = [
        (1_000_000_000, 10_000_000_000),
        (1_100_000_000, 10_100_000_000),
        (1_500_000_000, 10_200_000_000),
    ]
    summary = Probe.summarize_channel(rows, 1_600_000_000)
    assert summary['max_gap_ms'] == 400.
    assert summary['max_gap_interval']['measurement_before_ns'] == 10_100_000_000
    assert summary['max_gap_interval']['measurement_after_ns'] == 10_200_000_000
    assert summary['measurement_gap_ms']['max'] == 100.


def test_channel_summary_flags_nonmonotonic_measurement_timestamps():
    summary = Probe.summarize_channel([
        (1_000_000_000, 10_000_000_000),
        (1_100_000_000, 9_900_000_000),
        (1_200_000_000, 10_100_000_000),
    ], 1_300_000_000)
    assert summary['measurement_gap_ms']['nonpositive_count'] == 1
    assert summary['measurement_gap_ms']['max'] == 200.


def test_header_stamp_conversion_is_nanosecond_exact():
    class Stamp:
        sec = 17
        nanosec = 123_456_789

    class Message:
        class Header:
            stamp = Stamp()
        header = Header()

    assert stamp_ns(Message()) == 17_123_456_789
