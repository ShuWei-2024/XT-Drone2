"""Verify process isolation without starting PX4 or Gazebo."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'px4_launcher', REPO / 'xtd2_launch/xtd2_launch/utils/px4_launch.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_new_run_directory_and_selected_binary(self):
        with tempfile.TemporaryDirectory() as parent:
            checkout = Path(parent) / 'px4 checkout'
            binary = checkout / 'build/px4_sitl_default/bin/px4'
            binary.parent.mkdir(parents=True)
            binary.touch()
            run = Path(parent) / 'isolated run'
            process = Mock(returncode=7)
            process.poll.return_value = 7
            argv = ['px4_launch', '--model', 'gz_astra_x500_px4', '--id', '0',
                    '--px4-dir', str(checkout), '--namespace', 'x500_0']
            with patch.object(launcher.sys, 'argv', argv), \
                 patch.object(launcher.tempfile, 'mkdtemp', return_value=str(run)), \
                 patch.object(launcher.subprocess, 'Popen', return_value=process) as popen:
                self.assertEqual(7, launcher.main())
            args, kwargs = popen.call_args
            self.assertEqual(str(binary), args[0][0])
            self.assertEqual(str(run), args[0][-1])
            self.assertEqual(run, kwargs['cwd'])
            self.assertEqual('x500_0', kwargs['env']['PX4_UXRCE_DDS_NS'])
            self.assertEqual('4001', kwargs['env']['PX4_SYS_AUTOSTART'])
            self.assertNotIn('shell', kwargs)
            self.assertFalse((checkout / 'build/px4_sitl_default/parameters.bson').exists())


if __name__ == '__main__':
    unittest.main()
