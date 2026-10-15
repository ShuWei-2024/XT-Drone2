"""A fresh previous pair permits only a bounded callback-order wait."""
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from measurement_pair_gate import MeasurementPairGate, recent_measurement_pair
from types import SimpleNamespace


def test_no_pair_cannot_be_manufactured_from_unmatched_data():
    g=MeasurementPairGate()
    assert g.select(None,1_000_000_000,0.) is None


def test_delayed_matching_callback_replaces_previous_pair():
    g=MeasurementPairGate();old=(1_000_000_000,1.,object());new=(1_100_000_000,2.,object())
    assert g.select(old,1_020_000_000,0.) is old
    assert g.select(None,1_120_000_000,.1) is old
    assert g.select(None,1_128_000_000,.108) is old
    assert g.select(new,1_128_000_000,.108) is new
    assert g.pending_since is None


def test_unmatched_frames_do_not_extend_deadline():
    g=MeasurementPairGate();g.select((1_000_000_000,0.,object()),1_000_000_000,0.)
    assert g.select(None,1_100_000_000,.1) is not None
    assert g.select(None,1_116_000_000,.116) is not None
    assert g.select(None,1_124_000_000,.124) is None


def test_stale_and_future_cached_measurements_are_rejected():
    for now_ns in [999_000_000,1_251_000_000]:
        g=MeasurementPairGate();g.select((1_000_000_000,0.,object()),1_000_000_000,0.)
        assert g.select(None,now_ns,.01) is None


def local(stamp_ns, valid=True):
    return SimpleNamespace(timestamp_sample=stamp_ns//1000,z_valid=valid,z=-1.)


def test_px4_sample_gap_uses_previous_real_pair_without_widening_tolerance():
    # Measured failure: latest LiDAR stamp 78.608 s falls between PX4
    # 78.580/78.632 s outputs, both more than 20 ms away.
    old=(78_508_000_000,.49);new=(78_608_000_000,.50)
    history=[local(78_516_000_000),local(78_580_000_000),local(78_632_000_000)]
    pair=recent_measurement_pair([old,new],history,78_680_000_000)
    assert pair[:2]==old
    assert pair[2].timestamp_sample==78_516_000


def test_previous_pair_expires_and_cannot_cover_sustained_missing_measurements():
    measurements=[(1_000_000_000,1.)];history=[local(1_008_000_000)]
    assert recent_measurement_pair(measurements,history,1_250_000_000) is not None
    assert recent_measurement_pair(measurements,history,1_251_000_000) is None


def test_pair_rejects_future_invalid_and_out_of_tolerance_samples():
    measurements=[(1_000_000_000,1.)]
    assert recent_measurement_pair(measurements,[local(1_000_000_000,False)],1_100_000_000) is None
    assert recent_measurement_pair(measurements,[local(1_021_000_000)],1_100_000_000) is None
    assert recent_measurement_pair(measurements,[local(1_000_000_000)],999_000_000) is None
    assert recent_measurement_pair(measurements,[local(1_010_000_000)],1_005_000_000) is None
    assert recent_measurement_pair([(1_000_000_000,float('nan'))],[local(1_000_000_000)],1_100_000_000) is None


def test_height_pair_ignores_yaw_epoch_but_never_uses_previous_height_epoch():
    old=local(1_000_000_000);old.z_reset_counter=2;old.heading_reset_counter=1
    yaw_reset=local(1_100_000_000);yaw_reset.z_reset_counter=2;yaw_reset.heading_reset_counter=2
    values=[(1_000_000_000,1.)]
    pair=recent_measurement_pair(values,[old,yaw_reset],1_120_000_000,z_reset_counter=2)
    assert pair[2] is old
    assert recent_measurement_pair(values,[old,yaw_reset],1_120_000_000,z_reset_counter=3) is None


def test_nearer_expired_or_future_sample_cannot_mask_eligible_real_sample():
    values=[(1_006_000_000,1.)]
    history=[local(998_000_000),local(1_022_000_000)]
    assert recent_measurement_pair(values,history,1_250_000_000)[2] is history[1]
    assert recent_measurement_pair(values,history[:1],1_250_000_000) is None
    values=[(1_248_000_000,1.)]
    history=[local(1_251_000_000),local(1_236_000_000)]
    assert recent_measurement_pair(values,history,1_250_000_000)[2] is history[1]
