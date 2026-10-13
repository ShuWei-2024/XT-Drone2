"""Exercise the flight handoff gate with synthetic clocks; no PX4 output."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

source = Path(__file__).resolve().parents[1]/'ego_stability_20261010/ego_sitl_flight.py'
spec = importlib.util.spec_from_file_location('yaw_flight_test', source)
flight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flight)


class GateTest(unittest.TestCase):
    def run_gate(self, angle_deg=2., duplicate_measurement=False, pair_error_ns=0,
                 stale=False):
        self.now = 0.
        self.events = []
        node = NS(phase='', event=lambda name, **kwargs: self.events.append((name,kwargs)),
                  check_safe=lambda: None, send=lambda target: None,
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(self.now*1e9))))
        bridge = NS(odom=None, px4_history=[], odom_wall=None, px4_wall=None,
                    fresh=lambda wall, limit: wall is not None and self.now-wall <= limit)

        def receive(executor):
            self.now += .1
            stamp_ns = 100_000_000 if duplicate_measurement else int(self.now*1e9)
            sec,nsec = divmod(stamp_ns,10**9)
            bridge.odom = NS(header=NS(stamp=NS(sec=sec,nanosec=nsec)),
                             pose=NS(pose=NS(orientation=NS(x=0.,y=0.,z=0.,w=1.))))
            bridge.px4_history=[NS(timestamp_sample=(stamp_ns+pair_error_ns)//1000,
                                   heading=math.pi/2-math.radians(angle_deg),
                                   heading_good_for_control=True)]
            bridge.odom_wall=bridge.px4_wall=self.now-(.3 if stale else 0.)

        with patch.object(flight,'receive_batch',side_effect=receive), \
             patch.object(flight.time,'monotonic',side_effect=lambda: self.now):
            flight.await_initial_yaw_agreement(node,bridge,None,(0.,0.,-1.7,0.),5.)

    def test_good_paired_samples_require_two_seconds(self):
        self.run_gate()
        name,details=self.events[-1]
        self.assertEqual(name,'initial_yaw_agreement_complete')
        self.assertGreaterEqual(details['continuous_sec'],2.)

    def test_run42_size_initial_disagreement_times_out(self):
        with self.assertRaisesRegex(RuntimeError,'initial paired yaw agreement'):
            self.run_gate(angle_deg=14.4)
        self.assertFalse(any(e[0]=='initial_yaw_agreement_complete' for e in self.events))

    def test_duplicate_measurement_cannot_complete(self):
        with self.assertRaises(RuntimeError):
            self.run_gate(duplicate_measurement=True)

    def test_unpaired_or_stale_samples_cannot_complete(self):
        for kwargs in (dict(pair_error_ns=21_000_000),dict(stale=True)):
            with self.subTest(**kwargs),self.assertRaises(RuntimeError):
                self.run_gate(**kwargs)


if __name__=='__main__':
    unittest.main()
