"""Validate the prepared outdoor envelope before starting isolated SITL."""
import math


def outdoor_envelope(precheck, metadata):
    radius=metadata.get('interactive_goal_radius_m',10.)
    side=precheck.get('planner_map_size_xy_m',32.)
    for value,lo,hi in ((radius,3.,45.),(side,32.,80.)):
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
    if metadata.get('whole_map_selection', False):
        if (precheck.get('whole_map_selection') is not True or
                metadata.get('scene_edge_reserve_m') != 5. or
                precheck.get('scene_edge_reserve_m') != 5. or extent > 30. or
                radius < math.hypot(*[max(abs(bounds[0][i]),abs(bounds[1][i])) for i in range(2)])):
            raise ValueError('Whole-map selection requires matching geometry bounds and edge reserve')
    elif radius > 25. or side > 64.:
        raise ValueError('Expanded limits require explicit whole-map preparation')
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
    transit = 'transit_xy_boxes_m' in metadata
    if transit:
        # Full box schema is also checked when loading the immutable guard NPZ.
        certificate = metadata.get('transit_certificate', {})
        band = metadata.get('transit_body_z_bounds_m', [])
        if (certificate.get('sampled_bounds_m') != bounds or
                certificate.get('certified_z_bounds_m') != band or len(band) != 2 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in band) or
                not metadata['allowed_body_z_bounds_m'][0] <= band[0] < band[1] <= metadata['allowed_body_z_bounds_m'][1] or
                metadata.get('recovery_policy') != 'planned_return_with_certified_rtl' or
                metadata.get('transit_continuous_clearance_lower_bound_m',0) < .65 or
                certificate.get('direct_home_rectangle_reserve_m',0) < .2 or
                precheck.get('static_descent_columns') is not False or
                not band[0]+.2 <= precheck.get('takeoff_height_m',0)+.24 <= band[1]-.1):
            raise ValueError('Transit requires a cruise certificate and bounded native RTL policy')
        for key in ('transit_xy_boxes_m','transit_body_z_bounds_m','transit_certificate','recovery_policy',
                    'transit_includes_descent_region'):
            if precheck.get(key) != metadata.get(key):
                raise ValueError('Transit preparation differs between precheck and guard')
    return dict(goal_radius_m=float(radius),planner_map_size_xy_m=float(side),
                column_obstacle_max_z_m=float(maximum),
                transit_enabled=transit,
                supervisor_radius_m=math.sqrt(2)*extent+2.)
