"""Check that preflight separates receipt delay from invalid measurement time."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import threading
import time

spec = importlib.util.spec_from_file_location('health_probe_117',
    Path(__file__).resolve().parents[1] / 'tools/health_probe_117.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture(receipt_age_ms=28., sample_after_publish=False, truth=True):
    now = time.monotonic()
    sample = 10_000_000_000
    probe = module.Probe.__new__(module.Probe)
    probe.rows={k: [(int((now-.1+i*.005)*1e9), None) for i in range(20)] for k in
              ('lidar','camera','imu','odom','ready','px4_local','px4_status','gps','battery','px4_imu')}
    probe.rows_lock=threading.Lock()
    probe.ready=[(now-.01,True)]
    probe.local=SimpleNamespace(xy_valid=True,z_valid=True)
    probe.status=None
    probe.odom_ages=[24.]
    probe.px4_ages=[receipt_age_ms]
    probe.px4_timing=[(sample+int(receipt_age_ms*1e6), sample,
                       sample+(-4_000_000 if sample_after_publish else 4_000_000))]
    probe.clock_ns=probe.clock_receipt_ns=None
    probe.truth_lock=threading.Lock()
    probe.truth_times=[sample-8_000_000,sample+8_000_000] if truth else []
    return probe


def test_delivery_delay_is_not_clock_offset():
    result = module.Probe.summary(fixture())
    assert result['passed']
    assert result['px4_sample_to_publish']['p95_ms'] == 4.
    assert result['px4_truth_pair_dt']['p95_ms'] == 8.


def test_callback_order_does_not_invent_future_measurement():
    assert module.Probe.summary(fixture(receipt_age_ms=-204.))['passed']


def test_true_future_sample_and_missing_truth_fail():
    assert not module.Probe.summary(fixture(sample_after_publish=True))['passed']
    assert not module.Probe.summary(fixture(truth=False))['passed']


def test_stale_measurement_fails():
    assert not module.Probe.summary(fixture(receipt_age_ms=280.))['passed']
