"""
XTDrone2 PX4 Launch File

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn

Copyright (c) 2025 XTDrone2

This file is part of XTDrone2. XTDrone2 is free software: you can redistribute it
and/or modify it under the terms of the MIT License.

You should have received a copy of the MIT License along with XTDrone2. If not,
see <https://opensource.org/licenses/MIT>.
"""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

def main():
    parser = argparse.ArgumentParser(description='Launch px4 sitl in XTDrone2')

    parser.add_argument('--model', type=str, help='Model name', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id', required=True)
    parser.add_argument('--x', type=float, help='X position', required=False, default=0)
    parser.add_argument('--y', type=float, help='Y position', required=False, default=0)
    parser.add_argument('--z', type=float, help='Z position', required=False, default=0)
    parser.add_argument('--roll', type=float, help='Roll', required=False, default=0)
    parser.add_argument('--pitch', type=float, help='Pitch', required=False, default=0)
    parser.add_argument('--yaw', type=float, help='Yaw', required=False, default=0)
    parser.add_argument('--world', type=str, help='World name', required=False, default="default")
    parser.add_argument('--px4-dir', type=str, help='PX4 directory', required=False, default="~/PX4-Autopilot")
    parser.add_argument('--work-dir', type=str, default='',
                        help='Independent parameter/log directory; a new temp directory by default')
    parser.add_argument('--attach-model', default='', help='Use an existing Gazebo model')
    parser.add_argument('--startup-script', default='', help='SITL startup script in selected checkout')
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")

    args, unknown = parser.parse_known_args()

    autostart_map = {"gz_x500": 4001, "gz_x500_depth":4002,
                     "gz_astra_x500_px4": 4001}
    sys_autostart = autostart_map[args.model]

    mav_sys_id = args.id + 1

    if args.namespace:
        ns = args.namespace
    else:
        if args.model.startswith("gz_"):
            ns = f"{args.model[3:]}_{args.id}"
        else:
            ns = f"{args.model}_{args.id}"


    px4_dir = Path(args.px4_dir).expanduser().resolve()
    build_dir = px4_dir / 'build/px4_sitl_default'
    binary = build_dir / 'bin/px4'
    if not binary.is_file():
        parser.error(f'Build the selected PX4 checkout first: {binary}')
    work_dir = (Path(args.work_dir).expanduser().resolve() if args.work_dir
                else Path(tempfile.mkdtemp(prefix='astra-px4-')))
    work_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(PX4_UXRCE_DDS_NS=ns, PX4_GZ_WORLD=args.world,
               PX4_SYS_AUTOSTART=str(sys_autostart), PX4_SIM_MODEL=args.model,
               PX4_GZ_MODEL_POSE=','.join(str(v) for v in
                   (args.x, args.y, args.z, args.roll, args.pitch, args.yaw)),
               PX4_GZ_STANDALONE='1')
    if args.attach_model:
        env['PX4_GZ_MODEL_NAME'] = args.attach_model
    startup = (Path(args.startup_script).expanduser().resolve() if args.startup_script
               else build_dir / 'etc/init.d-posix/rcS')
    px4_cmd = [str(binary), '-d', '-s', str(startup),
               str(build_dir / 'etc'), '-i', str(args.id),
               '-w', str(work_dir)]
    print(f'PX4 checkout: {px4_dir}; parameters and logs: {work_dir}', flush=True)
    _handle = subprocess.Popen(px4_cmd, env=env, cwd=work_dir)

    try:
        while True:
            if _handle.poll() is not None:
                return _handle.returncode
            else:
                _handle.wait()
    except KeyboardInterrupt:
        _handle.terminate()
        try:
            _handle.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _handle.kill()
            _handle.wait()
        return 130



if __name__ == '__main__':
    sys.exit(main())
