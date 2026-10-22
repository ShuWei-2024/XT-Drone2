"""Separate startup, empty cuts, buffered clouds and supervisor receipt order."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from audit_livo_cut_timing import fault_context, image_shift_ns, summarize_cuts, visual_update_summary


def cut(sequence, stamp, output, **extra):
    return dict(sequence=str(sequence), stage='LIO', measurement_ns=str(stamp),
                odom_publish_ns=str(stamp + 40 if output else -1),
                processing_us='200', **extra)


def selected(lidar, image):
    return dict(event='publish_done', stamp_ns=str(lidar), image_ns=str(image), wall_ns='1')


def test_buffered_previous_cloud_can_publish_despite_early_image():
    summary, _ = summarize_cuts(
        [cut(1, 96, False), cut(2, 196, True),
         cut(3, 296, False, odom_output_status='empty_cut'), cut(4, 396, True)],
        [selected(t, t + 8) for t in (100, 200, 300, 400)], -12)
    assert summary['counts']['before_first_publication_rows'] == 1
    assert summary['counts']['no_output_total'] == 2
    assert summary['counts']['early_cut_after_first_publication'] == 3
    assert summary['counts']['early_no_output_after_first_publication'] == 1
    assert summary['counts']['confirmed_empty_cut_after_first_publication'] == 1


def test_no_output_is_not_attributed_to_nearest_cloud_without_image_match():
    summary, joined = summarize_cuts([cut(1, 208, True), cut(2, 308, False)],
                                    [selected(200, 208)], 0)
    assert summary['counts']['no_output_after_first_publication'] == 1
    assert summary['counts']['early_no_output_after_first_publication'] == 0
    assert 'selected_lidar_ns' not in joined[1]


def test_all_initialization_and_ambiguous_images_remain_unclassified():
    summary, _ = summarize_cuts([cut(1, 108, False)],
                                [selected(100, 108), selected(100, 108)], 0)
    assert summary['counts']['after_first_publication_rows'] == 0
    assert summary['counts']['ambiguous_adapter_image_rows'] == 1
    assert not summary['no_output_examples']


def test_supervisor_does_not_inherit_monitor_callback_and_end_is_not_publish():
    callbacks = [dict(label=label, wall_start_ns=start, wall_end_ns=start + 1,
                      measurement_ns=stamp) for label, start, stamp in (
        ('px4_calibration_flight:/fast_livo2/odom', 800, 700),
        ('other:/fast_livo2/odom', 990, 900),
        ('px4_calibration_flight:/fast_livo2/odom', 1002, 900))]
    missing = dict(measurement_ns=800, odom_publish_ns=-1)
    published = dict(measurement_ns=900, odom_publish_ns=950,
                     processing_end_wall_ns=1005)
    result = fault_context([dict(event='test_aborted', wall_ns=1000, sim_ns=960)],
                           callbacks, [missing, published])
    assert result['last_supervisor_callback']['measurement_ns'] == 700
    assert result['no_output_cuts_since_last_supervisor_measurement'] == [missing]
    assert result['next_published_measurement'] == published
    assert result['next_supervisor_callback_after_abort_ms'] == .000002


def test_effective_shift_uses_config_and_rejects_unlogged_online_calibration():
    params = dict(time_offset=dict(img_time_offset=-.012, exposure_time_init=.002),
                  rolling_shutter=dict(enable=True, anchor_factor=.25, tr_readout=.004))
    config = {'/**': {'ros__parameters': params}}
    assert image_shift_ns(config) == -9_000_000
    params['rolling_shutter']['td_estimate_enable'] = True
    with pytest.raises(ValueError, match='per-cut'):
        image_shift_ns(config)


def test_rejected_proposal_is_not_counted_as_an_applied_visual_update():
    fields = ('x', 'y', 'z', 'yaw', 'vx', 'vy', 'vz', 'bax', 'bay', 'baz',
              'gravity_x', 'gravity_y', 'gravity_z')
    def visual(rejected, dz, patches):
        row = {prefix + key: '0' for prefix in ('imu_', 'update_') for key in fields}
        return dict(row, stage='VIO', vio_update_rejected=str(rejected),
                    vio_accepted_patches=str(patches), update_z=str(dz),
                    vio_proposed_planar_correction_m='0.5')
    report = visual_update_summary([visual(1, 0., 400), visual(0, 0., 0),
                                    visual(0, .001, 300)])
    assert report['rejected_rows'] == 1
    assert report['observed_update_rows'] == 1
    assert report['rejected_with_observed_update_rows'] == 0
    assert report['non_rejected_with_accepted_patches_rows'] == 1
    assert report['maximum_observed_z_update_m'] == .001
