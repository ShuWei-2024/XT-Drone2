"""Bounded mission queue shared by UI commands and the sole flight supervisor."""
import math


class Mission:
    def __init__(self):
        self.goals = []
        self.started = False
        self.accepting = False
        self.closed = False
        self.paused = False
        self.completed = 0
        self.end_action = 'land'
        self.wait_seconds = 60.

    def append(self, delta):
        if not self.accepting or self.closed:
            raise ValueError('Waypoint editing is not open')
        if len(self.goals) >= 8:
            raise ValueError('This session allows at most 8 waypoints')
        if len(delta) != 2 or not all(math.isfinite(v) for v in delta):
            raise ValueError('Invalid waypoint')
        previous = self.goals[-1] if self.goals else [0., 0.]
        if math.dist(previous, delta) < .15:
            raise ValueError('Waypoints must be at least 0.15 m apart')
        self.goals.append(list(delta))

    def command(self, command, payload):
        if self.closed:
            raise ValueError('Mission is closing; restart after landing')
        if command == 'start':
            if self.started or not self.accepting or not self.goals:
                raise ValueError('Wait for READY and add at least one waypoint')
            action = payload.get('end_action', 'land')
            seconds = payload.get('wait_seconds', 60.)
            if action not in ('land', 'hold') or isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or not 30. <= seconds <= 120.:
                raise ValueError('Choose land/hold and a 30..120 s waiting timeout')
            self.end_action, self.wait_seconds = action, float(seconds)
            self.started = True
            self.accepting = False  # locked during takeoff/alignment
        elif command in ('clear', 'remove_last'):
            if self.started:
                raise ValueError('Active waypoints cannot be removed')
            if command == 'clear':
                self.goals.clear()
            elif self.goals:
                self.goals.pop()
        elif command in ('pause', 'resume'):
            if not self.started or not self.accepting:
                raise ValueError('Pause/resume requires active mission execution')
            self.paused = command == 'pause'
        else:
            raise ValueError('Unknown mission command')

    def close(self):
        self.closed = True
        self.accepting = False
