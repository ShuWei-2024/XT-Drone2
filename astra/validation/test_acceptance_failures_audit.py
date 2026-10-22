"""Evidence boundaries for acceptance failure reconstruction."""
import csv
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from audit_acceptance_failures_117 import Measurements, describe, largest_gap, height


def test_pairing_keeps_twenty_ms_limit_and_handles_missing_stream():
    assert Measurements([]).nearest(1) is None
    samples=Measurements([dict(measurement_time_ns='100000000')])
    assert samples.nearest(120_000_000) is not None
    assert samples.nearest(120_000_001) is None


def test_largest_gap_uses_requested_exact_publish_field():
    rows=[dict(odom_publish_end_wall_ns='110000000',processing_end_wall_ns='160000000'),
          dict(odom_publish_end_wall_ns='10000000',processing_end_wall_ns='20000000')]
    assert largest_gap(rows,'odom_publish_end_wall_ns')['gap_ms']==100.
    assert largest_gap(rows,'processing_end_wall_ns')['gap_ms']==140.


def test_percentile_interpolates_without_changing_units():
    assert describe([0.,100.])['p95']==pytest.approx(95.)
    assert describe([])==dict(samples=0)


def test_height_uses_ground_reference_and_excludes_post_abort_receipts(tmp_path):
    def write_csv(name, rows):
        with (tmp_path/name).open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader();writer.writerows(rows)
    events=[dict(event='ground_height_alignment_complete',sim_ns=10_000_000_000),
        dict(event='ego_control_handoff',sim_ns=20_000_000_000,wall_ns=20_000_000_000),
        dict(event='test_aborted',sim_ns=30_000_000_000,wall_ns=30_000_000_000)]
    (tmp_path/'motion_events.json').write_text(json.dumps(dict(events=events,fault='height drift',
        ground_height_alignment=dict(offset_m=-.1))))
    frames=[]
    for second in (8,9,10,29):
        for kind,z in (('gazebo',0. if second<20 else 1.),
                       ('fast_livo2',-.3 if second<20 else .545),
                       ('px4',.2 if second<20 else -.846)):
            frames.append(dict(source=kind,z_m=z,measurement_time_ns=second*10**9,
                               receipt_monotonic_ns=second*10**9))
    frames.append(dict(source='fast_livo2',z_m=-100.,measurement_time_ns=30_000_000_000,
                       receipt_monotonic_ns=31_000_000_000))
    write_csv('frame_samples.csv',frames)
    write_csv('livo_timing.csv',[dict(stage='LIO',measurement_ns=29_000_000_000,
        processing_end_wall_ns=29_000_000_000,update_z=.545,imu_z=.545,lio_correspondences=4000)])
    report=height(tmp_path)
    last=report['last_received_measurement']
    assert last['measurement_ns']==29_000_000_000
    assert last['livo_ground_referenced_error_m']==pytest.approx(-.155)
    assert last['px4_ground_referenced_error_m']==pytest.approx(.046)
    assert last['frozen_height_residual_m']==pytest.approx(-.201)
    assert report['decomposition_identity_max_error_m']<1e-12
