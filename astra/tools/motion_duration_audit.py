"""Count wall-clock seconds with independently observed vehicle motion."""
import math


class MotionDurationCounter:
    """Bounded-memory movement clock based on Gazebo truth samples.

    A sample interval counts only while the executor owns a fresh planned
    trajectory, the truth displacement is nonzero, and both sampling cadence
    and inferred speed remain inside conservative bounds. Pauses, endpoint
    holds, stale gaps, and discontinuous truth jumps do not count.
    """

    def __init__(self, minimum_speed_mps=.05, maximum_speed_mps=1.,
                 maximum_interval_s=.20):
        values=(minimum_speed_mps,maximum_speed_mps,maximum_interval_s)
        if (not all(math.isfinite(x) for x in values) or minimum_speed_mps<0 or
                maximum_speed_mps<=minimum_speed_mps or maximum_interval_s<=0):
            raise ValueError('Invalid truth-motion counter limits')
        self.minimum_speed_mps=minimum_speed_mps
        self.maximum_speed_mps=maximum_speed_mps
        self.maximum_interval_s=maximum_interval_s
        self.previous=None
        self.moving_seconds=0.
        self.moving_distance_m=0.
        self.moving_intervals=0

    def update(self, wall_ns, position, enabled=True):
        if (type(wall_ns) is not int or wall_ns<0 or len(position)<2 or
                not all(math.isfinite(float(v)) for v in position)):
            raise ValueError('Invalid truth motion sample')
        position=tuple(float(v) for v in position)
        speed=None; counted=0.
        if self.previous is not None and enabled:
            old_wall,old_position=self.previous
            dt=(wall_ns-old_wall)/1e9
            distance=math.dist(position,old_position)
            if 0.<dt<=self.maximum_interval_s:
                speed=distance/dt
                if self.minimum_speed_mps<=speed<=self.maximum_speed_mps:
                    counted=dt
                    self.moving_seconds+=dt
                    self.moving_distance_m+=distance
                    self.moving_intervals+=1
        self.previous=(wall_ns,position)
        return dict(speed_mps=speed,counted_seconds=counted,
                    moving_seconds=self.moving_seconds,
                    moving_distance_m=self.moving_distance_m,
                    moving_intervals=self.moving_intervals)


def expand_bounded_route(route, repeat_cycles, maximum_goals=64):
    """Repeat a prevalidated short route under an explicit finite test bound."""
    if (not isinstance(route,list) or not route or
            type(repeat_cycles) is not int or not 1<=repeat_cycles<=32 or
            type(maximum_goals) is not int or maximum_goals<=0 or
            len(route)*repeat_cycles>maximum_goals):
        raise ValueError('Repeated route exceeds its finite test bound')
    if (len(route)<2 or any(not isinstance(point,(list,tuple)) or len(point)!=2 or
            not all(math.isfinite(float(value)) for value in point) for point in route) or
            max(math.dist(route[index],route[(index+1)%len(route)])
                for index in range(len(route)))<.5):
        raise ValueError('Repeated route must contain distinct finite XY goals')
    return [list(point) for _ in range(repeat_cycles) for point in route]
