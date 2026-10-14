"""Frozen-reference drift and strict timestamp pairing must survive phase analysis."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from audit_long_route_drift import ENU_FROM_NED, phase_windows, pose_residuals, residual_summary


def row(t, xyz):
    return dict(measurement_time_ns=str(t), **dict(zip(('x_m', 'y_m', 'z_m'), map(str, xyz))))


def test_frozen_ground_reference_keeps_accumulated_height_drift():
    truth = [row(t*1_000_000_000, [0, 0, 1]) for t in (10, 12, 14)]
    source = [row(t*1_000_000_000, [0, 0, .8+drift])
              for t, drift in zip((10, 12, 14), (.05, .10, .20))]
    values = pose_residuals(source, truth, [0, 0, .2], np.eye(3))
    report = residual_summary(*values, len(source))
    assert np.isclose(report['z_m']['maximum'], .20)
    assert np.isclose(report['signed_z_first_second_m'], .05)
    assert np.isclose(report['signed_z_last_second_m'], .20)
    assert np.isclose(report['signed_z_change_m'], .15)


def test_ned_axis_swap_z_sign_and_ground_translation():
    source = [row(1_000_000_000, [3, 2, -1])]
    truth = [row(1_000_000_000, [2.1, 3.2, 1.3])]
    _, residual, dt = pose_residuals(source, truth, [.1, .2, .3], ENU_FROM_NED)
    np.testing.assert_allclose(residual, 0, atol=1e-12)
    np.testing.assert_array_equal(dt, [0])


def test_pair_limit_rejects_measurements_over_twenty_ms():
    truth = [row(1_000_000_000, [0, 0, 0])]
    source = [row(1_020_000_000, [0, 0, 0]), row(1_020_000_001, [0, 0, 0])]
    values = pose_residuals(source, truth, [0, 0, 0], np.eye(3))
    report = residual_summary(*values, len(source))
    assert report['paired_samples'] == 1
    assert report['pairing_fraction'] == .5
    assert report['truth_pair_dt_ms']['maximum'] == 20


def test_failure_before_handoff_is_waiting_not_executed_motion():
    windows = phase_windows([
        dict(event='ego_goal_sent', goal_index=0, sim_ns=100_000_000_000),
        dict(event='test_aborted', sim_ns=200_000_000_000),
        dict(event='land_requested', sim_ns=201_000_000_000),
        dict(event='disarmed', sim_ns=210_000_000_000)])
    assert 'ego_execution' not in windows
    assert windows['planner_wait'] == (100_000_000_000, 200_000_000_000)
    assert windows['landing'] == (201_000_000_000, 210_000_000_000)
