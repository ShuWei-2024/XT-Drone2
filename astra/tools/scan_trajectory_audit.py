"""Offline known-surface checks; this module sends no flight commands."""
import numpy as np
from scipy.interpolate import BSpline


def line_audit(guard, start, end, step_m=.01):
    start, end = np.asarray(start, float), np.asarray(end, float)
    length = float(np.linalg.norm(end-start))
    count = max(2, int(np.ceil(length/step_m))+1)
    points = np.linspace(start, end, count)
    distance = min(guard.clearance(p) for p in points)
    return dict(length_m=length, sampled_minimum_clearance_m=distance,
                continuous_clearance_lower_bound_m=distance-length/(count-1)/2,
                violates_clearance_margin=distance < guard.minimum_m)


def spline_audit(guard, message, offset, maximum_step_s=.25):
    knots = np.asarray(message['knots'], float)
    order = message['order']
    spline = BSpline(knots, message['controls'], order, extrapolate=False)
    begin, end = knots[order], knots[-order-1]
    if end <= begin:
        raise ValueError('Empty spline interval')
    velocity, acceleration = spline.derivative(1), spline.derivative(2)
    # Derivative control points bound motion between samples by convexity.
    speed_bound = float(np.linalg.norm(velocity.c, axis=1).max())
    acceleration_bound = float(np.linalg.norm(acceleration.c, axis=1).max())
    step=maximum_step_s
    while True:
        grid = np.linspace(begin, end, max(2, int(np.ceil((end-begin)/step))+1))
        clearance = min(guard.clearance(p) for p in spline(grid)+np.asarray(offset))
        dt = float(np.diff(grid).max())
        lower=clearance-speed_bound*dt/2
        # Coarse samples suffice for a conservative pass or an observed fail.
        # Resolve only the narrow uncertain band by refining to 5 ms.
        if lower>=guard.minimum_m or clearance<guard.minimum_m or step<=.005:break
        step=max(.005,step/2)
    return dict(start_ns=message['start_ns'],
                peak_sampled_speed_mps=float(np.linalg.norm(velocity(grid), axis=1).max()),
                peak_sampled_acceleration_mps2=float(np.linalg.norm(acceleration(grid), axis=1).max()),
                speed_bound_mps=speed_bound, acceleration_bound_mps2=acceleration_bound,
                sampled_minimum_clearance_m=clearance,
                continuous_clearance_lower_bound_m=clearance-speed_bound*dt/2,
                maximum_sample_step_s=dt)


def lateral_deviation(points, start, end):
    points, start, end = np.asarray(points), np.asarray(start), np.asarray(end)
    direction = end[:2]-start[:2]
    length = float(np.linalg.norm(direction))
    if not len(points) or length < 1e-9:
        return 0.
    normal = np.array([-direction[1], direction[0]])/length
    return float(np.abs((points[:, :2]-start[:2])@normal).max())
