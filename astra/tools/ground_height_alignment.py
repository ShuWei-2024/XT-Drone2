"""Freeze an estimator-to-estimator height offset, then verify on new samples."""
import math
import statistics


class GroundHeightAlignment:
    def __init__(self, duration_ns=4_000_000_000, limit_m=.15):
        self.duration_ns = duration_ns
        self.limit_m = limit_m
        self.calibration = []
        self.validation = []
        self.offset_m = None
        self.resets = None
        self.error = ''
        self.complete = False

    def add(self, stamp_ns, livo_z, px4_z, resets):
        if self.error or self.complete:
            return
        if not all(math.isfinite(v) for v in (livo_z, px4_z)) or stamp_ns <= 0:
            self.error = 'non-finite or invalid ground alignment sample'
            return
        resets = tuple(resets)
        if self.resets is None:
            self.resets = resets
        if resets != self.resets:
            self.error = 'PX4 reset during ground height alignment'
            return
        rows = self.validation or self.calibration
        if rows:
            delta = stamp_ns - rows[-1][0]
            if delta == 0:
                return
            if delta < 0 or delta > 250_000_000:
                self.error = 'ground alignment measurement gap/backwards time'
                return
        value = livo_z + px4_z
        if self.offset_m is None:
            self.calibration.append((stamp_ns, value))
            if stamp_ns-self.calibration[0][0] >= self.duration_ns and len(self.calibration) >= 30:
                self.offset_m = statistics.median(v for _,v in self.calibration)
                if max(abs(v-self.offset_m) for _,v in self.calibration) > self.limit_m:
                    self.error = 'ground height variation exceeds 0.15 m during capture'
        else:
            self.validation.append((stamp_ns,value))
            if abs(value-self.offset_m) > self.limit_m:
                self.error = 'ground height drift exceeds frozen offset by 0.15 m'
            elif stamp_ns-self.validation[0][0] >= self.duration_ns and len(self.validation) >= 30:
                self.complete = True

    def summary(self):
        return dict(offset_m=self.offset_m, complete=self.complete, error=self.error,
                    calibration_samples=len(self.calibration), validation_samples=len(self.validation),
                    maximum_validation_residual_m=max((abs(v-self.offset_m) for _,v in self.validation),default=None),
                    source='measurement-paired FAST-LIVO2 and PX4; no Gazebo calibration input',
                    frozen=True, calibration_seconds=self.duration_ns/1e9,
                    validation_seconds=self.duration_ns/1e9, limit_m=self.limit_m)
