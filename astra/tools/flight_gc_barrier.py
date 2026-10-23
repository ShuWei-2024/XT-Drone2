"""Bound the Python GC scan set during an armed SITL flight.

The barrier does not disable automatic collection and does not change any
flight freshness gate.  Only an explicitly isolated flight child may opt in
to process-wide freezing.  It collects once before arming, freezes objects
that are already initialized, and unfreezes/collects after disarm.  Objects
created by callbacks while airborne remain in the normal young generations.
The default is a no-op with respect to the process-wide freeze set.
"""

import gc
import time


class FlightGcBarrier:
    __slots__ = (
        'prepared', 'frozen', 'released', 'freeze_applied',
        'allow_process_wide',
        'was_enabled', 'thresholds',
        'freeze_count_before',
        'freeze_count_after_release',
        'prepare_wall_ns', 'release_wall_ns', 'prepare_collect_ns',
        'release_collect_ns', 'prepare_collected', 'release_collected',
    )

    def __init__(self, *, allow_process_wide=False):
        self.prepared = False
        self.frozen = False
        self.released = False
        self.freeze_applied = False
        self.allow_process_wide = bool(allow_process_wide)
        self.was_enabled = None
        self.thresholds = None
        self.freeze_count_before = None
        self.freeze_count_after_release = None
        self.prepare_wall_ns = None
        self.release_wall_ns = None
        self.prepare_collect_ns = None
        self.release_collect_ns = None
        self.prepare_collected = None
        self.release_collected = None

    def prepare(self, *, confirmed_disarmed):
        """Collect and freeze initialized objects before the first setpoint."""
        if self.prepared:
            return
        if not confirmed_disarmed:
            raise RuntimeError('GC preparation requires confirmed disarm')
        # Python exposes a process-wide freeze set, but has no API to remove
        # only objects added by this barrier.  A normal in-process caller must
        # therefore leave an existing set untouched.  The isolated SITL child
        # may opt in because it owns the whole interpreter and exits after the
        # run; this is recorded explicitly in the run evidence.
        self.freeze_count_before = gc.get_freeze_count()
        self.was_enabled = gc.isenabled()
        self.thresholds = gc.get_threshold()
        start = time.monotonic_ns()
        collect_start = time.monotonic_ns()
        self.prepare_collected = int(gc.collect(2))
        self.prepare_collect_ns = time.monotonic_ns() - collect_start
        # Keep automatic GC and its thresholds unchanged.  If this process
        # already had frozen objects, do not risk unfreezing another
        # component's process-wide freeze set during cleanup.
        if self.allow_process_wide:
            gc.freeze()
            self.freeze_applied = True
            self.frozen = True
        self.prepare_wall_ns = time.monotonic_ns() - start
        self.prepared = True
        self.released = False

    def release(self, *, confirmed_disarmed):
        """Restore the normal object generations after disarm."""
        if not self.prepared or self.released:
            return
        if not confirmed_disarmed:
            return
        start = time.monotonic_ns()
        if self.freeze_applied:
            gc.unfreeze()
        collect_start = time.monotonic_ns()
        self.release_collected = int(gc.collect(2))
        self.release_collect_ns = time.monotonic_ns() - collect_start
        self.release_wall_ns = time.monotonic_ns() - start
        self.released = True
        self.frozen = False
        self.freeze_count_after_release = gc.get_freeze_count()

    def summary(self):
        return {
            'prepared': self.prepared,
            'freeze_applied': self.freeze_applied,
            'process_wide_freeze_allowed': self.allow_process_wide,
            'frozen_during_armed_phase': self.freeze_applied,
            'released_after_disarm': self.released,
            'automatic_gc_enabled_before_prepare': self.was_enabled,
            'automatic_gc_enabled_now': gc.isenabled(),
            'thresholds_unchanged': self.thresholds is None or
                self.thresholds == gc.get_threshold(),
            'thresholds_before_prepare': self.thresholds,
            'freeze_count_before_prepare': self.freeze_count_before,
            'freeze_count_after_release': self.freeze_count_after_release,
            'prepare_collected': self.prepare_collected,
            'prepare_collect_ms': None if self.prepare_collect_ns is None else
                self.prepare_collect_ns / 1e6,
            'release_collected': self.release_collected,
            'release_collect_ms': None if self.release_collect_ns is None else
                self.release_collect_ns / 1e6,
        }
