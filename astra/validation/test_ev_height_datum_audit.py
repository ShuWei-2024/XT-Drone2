"""An independently frozen datum must not become an airborne residual fit."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from ev_height_datum_audit import fixed_origin_height_diagnostic


def diagnostic(errors, refs, **kwargs):
    values = dict(origin=dict(confirmed=True, origin_lla=[47., 8., .15],
                              confirmed_sim_us=30_000_000),
                  first_armed_us=80_000_000, height_reference=3)
    values.update(kwargs)
    return fixed_origin_height_diagnostic(errors, armed_ref_alt_m=refs, **values)


def test_known_datum_preserves_raw_error_and_does_not_fit_dynamic_drift():
    result = diagnostic([.15, .16, .40], [.15, .15])
    assert result['available'] and not result['certification']
    assert result['raw_z_m']['maximum'] == .40
    assert result['origin_referenced_z_m']['maximum'] == .25
    assert result['fixed_origin_altitude_m'] == .15


def test_negative_origin_uses_signed_offset():
    result = diagnostic([-.14, -.16], [-.15, -.15], origin=dict(
        confirmed=True, origin_lla=[47., 8., -.15], confirmed_sim_us=30_000_000))
    assert result['origin_referenced_z_m']['maximum'] == pytest.approx(.01)


@pytest.mark.parametrize('refs', [[.15, .16], [.25, .25], [], [np.nan]])
def test_changed_missing_or_mismatched_armed_origin_is_not_corrected(refs):
    result = diagnostic([.30], refs)
    assert not result['available']
    assert 'origin_referenced_z_m' not in result


@pytest.mark.parametrize('overrides', [
    dict(origin=dict(confirmed=False)),
    dict(first_armed_us=20_000_000),
    dict(first_armed_us=None),
    dict(height_reference=0),
])
def test_unconfirmed_airborne_origin_or_other_height_mode_is_not_corrected(overrides):
    assert not diagnostic([.30], [.15], **overrides)['available']
