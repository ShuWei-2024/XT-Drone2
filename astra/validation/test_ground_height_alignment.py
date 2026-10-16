"""Fixed origins must be allowed; drift and resets must not be fitted away."""
import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('ground_height_alignment',
    Path(__file__).resolve().parents[1]/'tools/ground_height_alignment.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fill(a,bias=.5,count=90,drift=False,reset=False):
    for i in range(count):
        value=bias+(.2 if drift and i>42 else .01*(i%3-1))
        a.add(1_000_000_000+i*100_000_000,0.,value,(0,1 if reset and i>42 else 0,0))
    return a


def test_constant_origin_bias_requires_separate_validation():
    a=fill(module.GroundHeightAlignment(),count=41)
    assert a.offset_m is not None and not a.complete
    frozen=a.offset_m
    for i in range(41,90):a.add(1_000_000_000+i*100_000_000,0.,.5,(0,0,0))
    assert a.complete and a.offset_m==frozen


def test_drift_is_rejected_without_refitting():
    a=fill(module.GroundHeightAlignment(),drift=True)
    assert a.error and not a.complete
    assert abs(a.offset_m-.5)<.02


def test_reset_and_measurement_gap_reject():
    assert fill(module.GroundHeightAlignment(),reset=True).error
    a=module.GroundHeightAlignment();a.add(1_000_000_000,0.,.5,(0,0,0))
    a.add(1_300_000_000,0.,.5,(0,0,0))
    assert a.error


def test_capture_variation_is_rejected():
    a=module.GroundHeightAlignment()
    for i in range(41):a.add(1_000_000_000+i*100_000_000,0.,.03*i,(0,0,0))
    assert a.error and not a.complete


def test_execution_retains_validated_ground_reference_despite_airborne_bias():
    a=fill(module.GroundHeightAlignment(),bias=.01)
    frozen=a.offset_m
    value,source=module.execution_height_reference(a,[.04,.045,.05])
    assert value==frozen and 'ground' in source
    assert a.offset_m==frozen
    # The existing 0.20 m gate must use this same origin in both consumers.
    assert abs(-.166-value)<.2
    assert abs(-.25-value)>.2


def test_execution_rejects_unvalidated_ground_reference_and_supports_legacy_capture():
    import pytest
    with pytest.raises(ValueError,match='validated'):
        module.execution_height_reference(module.GroundHeightAlignment(),[.04,.045,.05])
    value,source=module.execution_height_reference(None,[.04,.045,.05])
    assert value==.045 and 'airborne' in source
