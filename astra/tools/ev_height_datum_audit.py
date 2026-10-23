"""Offline PX4 1.17 height-datum diagnostic; never changes an input or flight gate.

EKF getPosition() publishes local_z = ref_alt - altitude. With EV as the
height reference, controlEvHeightFusion() references altitude to -EV_z.
Compare these only after accounting for the independently recorded origin.
Keep the raw discrepancy; this diagnostic is not frame certification.
"""
import numpy as np


def fixed_origin_height_diagnostic(signed_errors_m, origin, armed_ref_alt_m,
                                   first_armed_us, height_reference):
    report = dict(available=False, certification=False,
                  scope='PX4 1.17 EV height reference only; fixed disarmed origin, '
                        'no trajectory fit, bias estimate or flight-gate changes')
    if height_reference != 3:
        return dict(report, reason='EV is not the configured height reference')
    if not origin.get('confirmed'):
        return dict(report, reason='No confirmed disarmed origin record')
    try:
        altitude = origin['origin_lla'][2]
    except (KeyError, TypeError, IndexError):
        return dict(report, reason='Missing confirmed origin altitude')
    confirmed = origin.get('confirmed_sim_us')
    if (confirmed is None or first_armed_us is None or confirmed >= first_armed_us):
        return dict(report, reason='Origin was not confirmed before arming')
    try:
        altitude = float(altitude)
        refs = np.asarray(armed_ref_alt_m, dtype=float)
        errors = np.asarray(signed_errors_m, dtype=float)
    except (TypeError, ValueError):
        return dict(report, reason='Invalid origin, reference or residual data')
    if (not np.isfinite(altitude) or refs.ndim != 1 or errors.ndim != 1 or
            not len(refs) or not len(errors) or not np.isfinite(refs).all() or
            not np.isfinite(errors).all()):
        return dict(report, reason='Missing or nonfinite origin/reference/residual data')
    if np.max(np.abs(refs-altitude)) > 1e-4:
        return dict(report, reason='Armed ref_alt changed or differs from frozen origin')
    def describe(values):
        return dict(count=len(values), p95=float(np.percentile(values, 95)),
                    maximum=float(np.max(values)))
    return dict(report, available=True,
                fixed_origin_altitude_m=altitude,
                origin_confirmed_sim_us=confirmed,
                raw_z_m=describe(np.abs(errors)),
                origin_referenced_z_m=describe(np.abs(errors-altitude)),
                equation='local_z - EV_z - frozen_ref_alt',
                reason='Fixed disarmed origin matches every armed ULog ref_alt sample')
