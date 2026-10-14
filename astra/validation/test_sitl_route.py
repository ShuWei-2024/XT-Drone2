"""Reject invalid routes before starting simulation or issuing flight commands."""
import importlib.util
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))

spec = importlib.util.spec_from_file_location('run_sitl_117',
    Path(__file__).resolve().parents[1] / 'tools/run_sitl_117.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('content', [
    '[]', '[[true,0]]', '[[NaN,0]]', '[[8,0]]', '[[0,0,0]]',
    '[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]',
    '{"x": 1}',
])
def test_unsafe_route_rejected(tmp_path, content):
    path = tmp_path / 'route.json'
    path.write_text(content)
    with pytest.raises(ValueError):
        module.load_route(path)


def test_repeated_directions_preserved():
    path = Path(__file__).resolve().parents[1] / 'config/route_repeat_north_south.json'
    assert module.load_route(path) == [[5,3],[0,0],[5,-3],[0,0],[5,3],[0,0],[5,-3],[0,0]]


@pytest.mark.parametrize('args', [
    ['gz sim','-s','-r','world.sdf'], ['gz','sim','-s'],
    ['gz sim -s -r /home/cyf/dev/XT-Drone2-astra/astra/runtime/example/test_world.sdf'],
    ['ruby','/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz','sim','-s'],
    ['/path/to/px4'], ['/path/to/MicroXRCEAgent','udp4'],
])
def test_simulator_launcher_and_rewritten_title_detected(args):
    assert module.is_simulation_command(args)


@pytest.mark.parametrize('args', [[], ['gz','topic'], ['ruby','unrelated.rb'], ['bash']])
def test_unrelated_commands_are_not_simulators(args):
    assert not module.is_simulation_command(args)


def test_cleanup_signals_group_even_if_launcher_already_exited(monkeypatch):
    from types import SimpleNamespace
    import signal
    calls=[]
    monkeypatch.setattr(module.os,'killpg',lambda pid,sig:calls.append((pid,sig)))
    exited=SimpleNamespace(pid=1234,poll=lambda:0)
    module.signal_owned_group(exited,signal.SIGTERM)
    assert calls==[(1234,signal.SIGTERM)]


def test_absent_group_is_already_clean(monkeypatch):
    from types import SimpleNamespace
    import signal
    def absent(*args):raise ProcessLookupError()
    monkeypatch.setattr(module.os,'killpg',absent)
    module.signal_owned_group(SimpleNamespace(pid=1234),signal.SIGTERM)


@pytest.mark.parametrize('mode', ['timer', 'small', 'obstacle'])
def test_scan_rejects_unprepared_flight_modes_and_missing_indoor_route(monkeypatch, mode):
    monkeypatch.setattr(module.sys, 'argv', ['run_sitl_117', mode, 'scan_test',
                                           '--world-preset', 'replica-office0'])
    with pytest.raises(SystemExit) as exc:
        module.main()
    assert exc.value.code == 2
