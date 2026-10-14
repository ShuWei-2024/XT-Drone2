"""Validate the prepared outdoor envelope before starting isolated SITL."""
import math


def outdoor_envelope(precheck, metadata):
    radius=metadata.get('interactive_goal_radius_m',10.)
    side=precheck.get('planner_map_size_xy_m',32.)
    for value,lo,hi in ((radius,3.,25.),(side,32.,64.)):
        if type(value) not in (int,float) or not math.isfinite(value) or not lo<=value<=hi:
            raise ValueError('Invalid prepared outdoor radius or planner map size')
    if precheck.get('interactive_goal_radius_m')!=radius:
        raise ValueError('Outdoor radius differs between precheck and guard')
    bounds=metadata.get('allowed_xy_bounds_m')
    if (not isinstance(bounds,list) or len(bounds)!=2 or
            any(not isinstance(row,list) or len(row)!=2 for row in bounds) or
            any(type(v) not in (int,float) or not math.isfinite(v) for row in bounds for v in row)):
        raise ValueError('Invalid outdoor XY bounds')
    extent=max(abs(v) for row in bounds for v in row)
    if (any(bounds[0][i]>=0 or bounds[1][i]<=0 or bounds[0][i]>=bounds[1][i] for i in range(2))
            or extent>30. or side/2.<extent+5.):
        raise ValueError('Outdoor planner grid must cover the envelope plus 5 m local reserve')
    if bounds!=precheck.get('allowed_xy_bounds_m'):
        raise ValueError('Outdoor bounds differ between precheck and guard')
    certificate=metadata.get('recovery_certificate',{})
    if (certificate.get('sampled_bounds_m')!=bounds or
            certificate.get('certified_z_bounds_m')!=metadata.get('allowed_body_z_bounds_m') or
            metadata.get('recovery_continuous_clearance_lower_bound_m',0)<.65):
        raise ValueError('Outdoor descent certificate does not cover the prepared envelope')
    ceiling=precheck.get('planner_virtual_ceiling_m',1.5)
    maximum=precheck.get('column_obstacle_max_z_m',3. if radius>10. else ceiling)
    if (type(ceiling) not in (int,float) or not math.isfinite(ceiling) or
            type(maximum) not in (int,float) or not math.isfinite(maximum) or not ceiling<=maximum<=3.):
        raise ValueError('Invalid observed branch height range')
    return dict(goal_radius_m=float(radius),planner_map_size_xy_m=float(side),
                column_obstacle_max_z_m=float(maximum),
                supervisor_radius_m=math.sqrt(2)*extent+2.)
