"""Isolated XT-Drone2/PX4 1.17 integration runs; flight modes explicitly arm SITL."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

REPO=Path(__file__).resolve().parents[2]
PX4=Path(os.environ.get('ASTRA_PX4_DIR', str(Path.home()/'dev/px4/PX4-Autopilot-1.17')))


def load_route(path):
    route=json.loads(path.read_text())
    if (not isinstance(route,list) or not 1 <= len(route) <= 8 or
        any(not isinstance(xy,list) or len(xy)!=2 or
            not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                    for v in xy) or math.hypot(*xy)>7. for xy in route)):
        raise ValueError('Route must contain 1..8 finite XY offsets within 7 m of the frozen origin')
    return route


def is_simulation_command(args):
    if not args:return False
    if args[0]=='gz sim' or args[0].startswith('gz sim '):return True
    name=Path(args[0]).name
    if name in ('px4','MicroXRCEAgent','gz-sim-server','gz sim'):return True
    # Gazebo may run as ruby's gz launcher or change argv[0] to "gz sim".
    return ((name=='gz' and len(args)>1 and args[1]=='sim') or
            (name in ('ruby','ruby3.2') and len(args)>2 and Path(args[1]).name=='gz'
             and args[2]=='sim'))


def signal_owned_group(proc,sig):
    # A launch process can exit while its Gazebo child still owns this group.
    # poll() only describes the launcher, so it cannot gate group cleanup.
    try:os.killpg(proc.pid,sig)
    except ProcessLookupError:pass


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['timer','static','small','obstacle'])
    p.add_argument('name');p.add_argument('--seconds',type=float,default=60.)
    p.add_argument('--world-preset',choices=['geometry-rich','indoor-hall','replica-office0'],default='geometry-rich',
                   help='Replica supports static checks or the prepared guarded indoor obstacle route')
    p.add_argument('--calibrate-ground-height',action='store_true')
    p.add_argument('--lio-only-diagnostic',action='store_true',
                   help='Isolated comparison: disable image updates in the copied estimator config')
    p.add_argument('--lio-voxel-size-m',type=float,choices=[.25,.5],default=.5,
                   help='Isolated plane-map resolution comparison; default retains 0.5 m')
    p.add_argument('--plane-quality-diagnostic',action='store_true',
                   help='Copied config only: reject thick or line-like plane fits; subdivide rejected voxels')
    p.add_argument('--capture-scan-inputs',action='store_true',
                   help='Sparse raw cloud and IMU/pose capture for offline sensor geometry audit')
    p.add_argument('--executor-audit',action='store_true',help='Trace flight callbacks and DDS queue timestamps')
    p.add_argument('--execution-executor',choices=['ready-batch','events'],default='ready-batch',
                   help='Optional experimental Jazzy event executor; isolated obstacle SITL only')
    p.add_argument('--route-json',type=Path,
                   help='Obstacle test goal offsets; up to 8 goals, each bounded to 7 m')
    p.add_argument('--stop-map-after-handoff-s',type=float,
                   help='SITL fault regression: stop this run\'s map after planner control handoff')
    p.add_argument('--baro-noise-m',type=float,choices=[.1,.3],default=.3,
                   help='Isolated SITL EKF baro sigma: default 0.3 m matches the model\'s '
                        '3 Pa pressure noise (about 0.25 m); 0.1 m retains the old comparison')
    p.add_argument('--gps-delay-ms',type=float,choices=[0.,110.],default=110.,
                   help='Isolated single-factor GPS delay comparison; default preserves 110 ms')
    p.add_argument('--pressure-noise-pa',type=float,choices=[0.,.05,3.],default=3.,
                   help='Diagnostic noise ablation only; normal simulation retains 3 Pa. '
                        '0 Pa can trigger PX4 stuck-sensor detection; use 0.05 Pa for low-noise diagnosis')
    a=p.parse_args()
    if a.execution_executor=='events' and a.mode!='obstacle':
        p.error('Event executor candidate is restricted to isolated obstacle SITL')
    replica_dir=REPO/'astra/runtime/maps/replica_office0'
    if a.world_preset=='replica-office0':
        if a.mode not in ('static','obstacle'):
            p.error('Replica office0 requires static mode or its prepared mesh-guarded obstacle route')
        if a.mode=='obstacle' and not a.route_json:
            p.error('Replica flight requires an explicit prepared indoor route')
        if not (replica_dir/'manifest.json').is_file():
            p.error('Prepare the scan first with astra/tools/prepare_replica_office0.py')
        manifest=json.loads((replica_dir/'manifest.json').read_text())
        for asset,digest in manifest['asset_sha256'].items():
            if hashlib.sha256((replica_dir/'replica_office0/meshes'/asset).read_bytes()).hexdigest()!=digest:
                p.error('Converted Replica asset changed: '+asset)
    if a.route_json and a.mode!='obstacle':p.error('Custom routes require obstacle mode')
    try:
        route_offsets=load_route(a.route_json) if a.route_json else [[5,3],[0,0]]
    except (OSError,ValueError) as exc:p.error(str(exc))
    if a.world_preset=='replica-office0' and a.mode=='obstacle':
        try:
            flight_precheck=json.loads((replica_dir/'flight_precheck.json').read_text())
            if (route_offsets!=flight_precheck['route_offsets_m'] or
                flight_precheck['collision_sha256']!=manifest['asset_sha256']['collision.stl'] or
                hashlib.sha256((replica_dir/'world.sdf').read_bytes()).hexdigest()!=flight_precheck['world_sha256'] or
                hashlib.sha256((replica_dir/'replica_office0/model.sdf').read_bytes()).hexdigest()!=flight_precheck['model_sdf_sha256'] or
                hashlib.sha256((replica_dir/'mesh_guard.npz').read_bytes()).hexdigest()!=flight_precheck['guard_sha256']):
                p.error('Replica route or guard differs from the prepared flight precheck')
        except (OSError,ValueError) as exc:p.error('Prepare scan flight first: '+str(exc))
    flight_timeout=max(330.,150.+100.*len(route_offsets)) if a.mode=='obstacle' else 330.
    if a.stop_map_after_handoff_s is not None and (a.mode!='obstacle' or
            not 1. <= a.stop_map_after_handoff_s <= 10.):
        p.error('Map-stop regression requires obstacle mode and a delay of 1..10 seconds')
    if not re.fullmatch('[a-zA-Z0-9_-]{1,64}',a.name):p.error('Use a new short run name')
    if os.environ.get('ASTRA_117_ENV_LOADED')!='1':p.error('Use astra/run_117.bash to load the isolated ROS environment')
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=PX4,text=True).strip() != json.loads((REPO/'astra/dependencies.lock.json').read_text())['px4']['commit']:
        p.error('PX4 commit differs from the lock file')
    for patch in ('astra/baseline/px4_1_15_3_hrt.patch','astra/config/px4_117_gz_magnetometer.patch'):
        if subprocess.run(['git','apply','--reverse','--check',str(REPO/patch)],cwd=PX4,
                          stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode:
            p.error('Required isolated SITL patch absent or changed: '+patch)
    # Refuse overlapping simulators or an agent sharing the SITL port.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:args=(proc/'cmdline').read_bytes().split(b'\0')
        except (OSError,PermissionError):continue
        if is_simulation_command([x.decode(errors='replace') for x in args if x]):
            p.error(f'Existing simulation process {proc.name}; stop it first')
    out=REPO/'astra/runtime'/a.name;out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(Path(__file__),out/'runner.py')
    snapshot=out/'execution_sources';snapshot.mkdir()
    source_paths=list((REPO/'astra/tools').glob('*.py'))
    source_paths += list((REPO/'astra/packages/astra_px4_offboard/scripts').glob('*.py'))
    source_paths += list((REPO/'astra/packages/astra_fast_livo2/launch').glob('*.py'))
    source_paths += list((REPO/'astra/packages/astra_fast_livo2/scripts').glob('*.py'))
    execution_hashes={}
    for source in source_paths:
        relative=source.relative_to(REPO);dest=snapshot/relative
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        execution_hashes[str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()
    for relative in ('install_117/astra_fast_livo2_core/lib/astra_fast_livo2_core/fastlivo_mapping',
                     'install_117/ego_planner/lib/ego_planner/ego_planner_node'):
        execution_hashes[relative]=hashlib.sha256((REPO/relative).read_bytes()).hexdigest()
    (snapshot/'manifest.json').write_text(json.dumps(dict(captured='before simulator start',
        sha256=execution_hashes),indent=2)+'\n')
    shutil.copy2(REPO/'astra/config/estimator_self_filtered.yaml', out/'estimator_config.yaml')
    if a.lio_only_diagnostic or a.lio_voxel_size_m != .5 or a.plane_quality_diagnostic:
        import yaml
        config=yaml.safe_load((out/'estimator_config.yaml').read_text())
        if a.lio_only_diagnostic:config['/**']['ros__parameters']['common']['img_en']=0
        config['/**']['ros__parameters']['lio']['voxel_size']=a.lio_voxel_size_m
        if a.plane_quality_diagnostic:
            config['/**']['ros__parameters']['lio']['max_plane_eigen_ratio']=.1
            config['/**']['ros__parameters']['lio']['min_plane_extent_ratio']=.01
        (out/'estimator_config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    tools=REPO/'astra/tools';processes=[];streams=[]
    original=json.loads((REPO/'astra/baseline/original_state.json').read_text())
    before={path:hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in original['files']}
    (out/'original_parameters.sha256').write_text(''.join(
        f'{digest}  {path}\n' for path,digest in before.items() if path.endswith('.bson')))
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='94',GZ_PARTITION='astra_px4_117',ASTRA_PX4_NAMESPACE='x500_0',
        ASTRA_GZ_MAG_ENU_TESLA='1',
        FASTRTPS_DEFAULT_PROFILES_FILE=str(REPO/'astra/packages/astra_fast_livo2/config/fastdds_sitl.xml'))
    models=out/'models';src=REPO/'astra/packages/astra_gz_sensors/models'
    shutil.copytree(src,models)
    if a.world_preset=='replica-office0':
        shutil.copytree(replica_dir/'replica_office0',models/'replica_office0')
        scan_sdf=models/'replica_office0/model.sdf';scan_tree=ET.parse(scan_sdf)
        for texture in scan_tree.getroot().iter('albedo_map'):
            texture.text=str(models/'replica_office0'/texture.text)
        scan_tree.write(scan_sdf,encoding='utf-8',xml_declaration=True)
        shutil.copy2(replica_dir/'manifest.json',out/'map_manifest.json')
        shutil.copy2(replica_dir/'REPLICA_LICENSE.txt',out/'REPLICA_LICENSE.txt')
    # The official Harmonic model's magnetic noise is expressed in gauss.
    # This world's explicit tesla convention requires the same noise scaled
    # by 1e-4. Copy the base model into this run, preserving official sources.
    shutil.copytree(PX4/'Tools/simulation/gz/models/x500_base',models/'x500_base')
    magnetic_model=models/'x500_base/model.sdf';mag_tree=ET.parse(magnetic_model)
    for mag in mag_tree.getroot().iter('magnetometer'):
        for field in mag.iter():
            if field.tag in ('stddev','mean','bias_mean','bias_stddev','dynamic_bias_stddev') and field.text:
                field.text=str(float(field.text)*1.e-4)
    for pressure in mag_tree.getroot().iter('air_pressure'):
        pressure.find('pressure/noise/stddev').text=str(a.pressure_noise_pa)
    mag_tree.write(magnetic_model,encoding='utf-8',xml_declaration=True)
    sensor=models/'astra_x500_sensors/model.sdf';tree=ET.parse(sensor)
    for link in tree.getroot().iter('link'):
        for s in list(link.findall('sensor')):
            if s.get('type')=='depth_camera':link.remove(s)
    tree.write(sensor,encoding='utf-8',xml_declaration=True)
    source_world=(replica_dir/'world.sdf' if a.world_preset=='replica-office0' else
                  REPO/'astra/packages/astra_gz_sensors/worlds'/
                  ('astra_px4_indoor_hall.sdf' if a.world_preset=='indoor-hall' else 'astra_px4_geometry_rich.sdf'))
    world=out/'test_world.sdf';shutil.copy2(source_world,world)
    # Keep texture paths valid after copying the world.
    shutil.copytree(REPO/'astra/packages/astra_gz_sensors/worlds/textures',out/'textures')
    xml=ET.parse(world)
    for texture in xml.getroot().iter('albedo_map'):
        texture.text=str(out/'textures'/Path(texture.text).name)
    xml.write(world,encoding='utf-8',xml_declaration=True)
    if a.world_preset in ('geometry-rich','indoor-hall'):
        subprocess.run([sys.executable,str(tools/'obstacle_world_geometry_20261010.py'),str(world),
                        '--output',str(out/'obstacle_geometry.json')],check=True)
        env['ASTRA_TEST_OBSTACLE_GEOMETRY']=str(out/'obstacle_geometry.json')
    else:
        env.pop('ASTRA_TEST_OBSTACLE_GEOMETRY',None)
        if a.mode=='obstacle':
            shutil.copy2(replica_dir/'mesh_guard.npz',out/'mesh_guard.npz')
            shutil.copy2(replica_dir/'flight_precheck.json',out/'flight_precheck.json')
            env['ASTRA_TEST_MESH_GUARD']=str(out/'mesh_guard.npz')
    if a.world_preset in ('geometry-rich','indoor-hall') or a.mode=='static':
        env.pop('ASTRA_TEST_MESH_GUARD',None)
    startup=out/'startup.sh'
    overrides={'UXRCE_DDS_SYNCT':0,'EKF2_EV_CTRL':0,'EKF2_HGT_REF':0,'EKF2_GPS_CTRL':5,
               'EKF2_MAG_TYPE':1,'EKF2_BARO_NOISE':a.baro_noise_m,'EKF2_GPS_DELAY':a.gps_delay_ms,
               'EKF2_GND_EFF_DZ':0,'MPC_XY_P':1.2}
    env.update({f'PX4_PARAM_{key}':str(value) for key,value in overrides.items()})
    startup.write_text('. "${R}etc/init.d-posix/rcS"\n')
    runtime={'mode':a.mode,'world':str(world),'px4':str(PX4),'namespace':'x500_0',
             'world_preset':a.world_preset,'static_only_scan':a.world_preset=='replica-office0' and a.mode=='static',
             'mesh_guard_enabled':a.world_preset=='replica-office0' and a.mode=='obstacle',
             'lio_only_diagnostic':a.lio_only_diagnostic,
             'lio_voxel_size_m':a.lio_voxel_size_m,
             'plane_quality_diagnostic':a.plane_quality_diagnostic,
             'capture_scan_inputs':a.capture_scan_inputs,
             'executor_audit':a.executor_audit,
             'execution_executor':a.execution_executor,
             'px4_binary_sha256':hashlib.sha256((PX4/'build/px4_sitl_default/bin/px4').read_bytes()).hexdigest(),
             'magnetometer_convention':'Gazebo world ENU / tesla -> body FLU -> PX4 body FRD / gauss',
             'magnetometer_noise_scale_from_official_gauss_model':1.e-4,
             'route_offsets_m':route_offsets if a.mode=='obstacle' else None,
             'flight_timeout_s':flight_timeout,
             'pressure_noise_pa':a.pressure_noise_pa,
             'diagnostic_noiseless_pressure':a.pressure_noise_pa==0.,
             'diagnostic_reduced_pressure_noise':a.pressure_noise_pa!=3.,
             'sensors_depth_removed':True,'parameter_overrides':overrides,'isolated_rootfs':str(out),'ros_domain':94,'gz_partition':env['GZ_PARTITION']}
    runtime['odom_reference'] = 'base_link; internal LIVO state remains at imu_link'
    runtime['audit_truth_reference_offset_model_m'] = [0., 0., .24]
    runtime['truth_guard_reference'] = ('base_link with rotated model offset; scan mesh clearance and '
        '0.6 s measured-motion boundary forecast' if runtime['mesh_guard_enabled'] else
        'outer Gazebo model pose; conservative stop only, never flight input')
    (out/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
    subprocess.run([sys.executable,str(tools/'verify_sensor_geometry_117.py'),
                    '--models',str(models),'--px4',str(PX4),
                    '--config',str(out/'estimator_config.yaml'),
                    '--output',str(out/'sensor_geometry.json')],check=True)
    def start(name,cmd):
        stream=(out/(name+'.log')).open('w');streams.append(stream)
        proc=subprocess.Popen(cmd,env=env,cwd=REPO,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,proc));return proc
    def pause(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            for name,proc in processes:
                if name in ('agent','simulator','gcs_heartbeat','livo','map','planner') and proc.poll() is not None:
                    raise RuntimeError(f'{name} exited; inspect {out}/{name}.log')
            time.sleep(.2)
    result=1
    failure=''
    try:
        start('agent',[str(Path.home()/'dev/third_party/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),'udp4','-p','8888'])
        start('simulator',['ros2','launch','xtd2_launch','astra_sitl.launch.py',f'px4_dir:={PX4}',
             f'work_dir:={out}',f'startup_script:={startup}',f'world_file:={world}',f'model_resource_path:={models}'])
        start('gcs_heartbeat',[str(Path.home()/'dev/px4/venv/bin/python'),str(tools/'sitl_gcs_heartbeat.py')])
        pause(25.)
        if a.mode=='timer':
            for name in ('vehicle_gps_position','sensor_mag','battery_status'):
                r=subprocess.run([str(PX4/'build/px4_sitl_default/bin/px4-listener'),name,'-n','1'],
                                 env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=4.)
                (out/f'before_{name}.txt').write_bytes(r.stdout)
            r=subprocess.run([str(PX4/'build/px4_sitl_default/bin/px4-tests'),'hrt','reschedule'],
                             env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=5.)
            (out/'hrt_result.txt').write_bytes(r.stdout);print(r.stdout.decode(),flush=True)
            simulator_log=(out/'simulator.log').read_text()
            (out/'hrt_result.txt').write_text(r.stdout.decode()+simulator_log[simulator_log.rfind('HRT reschedule:'):].split('\n',1)[0]+'\n')
            result=0 if 'result=PASS' in simulator_log else 1
            pause(2.)
        else:
            start('livo',['ros2','launch','astra_fast_livo2','fast_livo2.launch.py',
                 f'estimator_config:={out}/estimator_config.yaml',
                 'point_time_mode:=snapshot','max_retained_images:=512','max_planar_correction_m:=0.04',
                 f'timing_audit_csv:={out}/livo_timing.csv',f'input_audit_csv:={out}/input_audit.csv'])
            map_proc=start('map',['ros2','run','astra_fast_livo2','local_mapping_guard.py','--ros-args',
                 '-p','use_sim_time:=true','-p','self_filter_radius_m:=0.45'])
            start('planner',['ros2','launch','astra_fast_livo2','ego_planner_guarded.launch.py',
                 'max_vel:=0.10' if a.world_preset=='replica-office0' else 'max_vel:=0.20',
                 'max_acc:=0.10' if a.world_preset=='replica-office0' else 'max_acc:=0.25',
                 'planning_horizon:=6.0','max_tracking_error_m:=0.60','virtual_ceiling_m:=2.3',
                 'min_goal_distance_m:=0.04' if a.world_preset=='replica-office0' else 'min_goal_distance_m:=0.20'])
            pause(12.)
            seconds=a.seconds if a.mode=='static' else flight_timeout+60.
            monitor=start('monitor',[sys.executable,str(tools/'gpu_joint_monitor.py'),'--output',str(out),
                 '--seconds',str(seconds),'--stop-file',str(out/'controller_finished')])
            audit=start('frame',[sys.executable,str(REPO/'astra/packages/astra_px4_offboard/scripts/audit_frame_alignment.py'),
                 '--seconds',str(seconds),'--px4-timebase','sim','--px4-namespace','x500_0',
                 '--truth-reference-offset','0','0','0.24',
                 '--output',str(out/'frame.json'),'--samples-output',str(out/'frame_samples.csv'),
                 '--stop-file',str(out/'controller_finished')])
            health=start('health',[sys.executable,str(tools/'health_probe_117.py'),'--seconds',str(seconds),
                 '--output',str(out/'health.json'),'--stop-file',str(out/'controller_finished')])
            scan_probe=None
            if a.world_preset=='replica-office0':
                scan_probe=start('scan_probe',[sys.executable,str(tools/'probe_scan_scene.py'),
                     '--seconds',str(seconds),'--output',str(out),
                     '--stop-file',str(out/'controller_finished')]+(['--capture-inputs'] if a.capture_scan_inputs else []))
            if a.mode=='static':
                pause(seconds+1.);health.wait(timeout=5.)
                result=0 if json.loads((out/'health.json').read_text())['passed'] else 1
                if scan_probe is not None:
                    scan_probe.wait(timeout=8.)
                    if not json.loads((out/'scan_probe.json').read_text())['passed']:result=1
            else:
                precheck=start('preflight_health',[sys.executable,str(tools/'health_probe_117.py'),
                    '--seconds','12','--output',str(out/'preflight_health.json')])
                if precheck.wait(timeout=20.) != 0 or not json.loads((out/'preflight_health.json').read_text())['passed']:
                    raise RuntimeError('Passive health gate failed; no arming commands sent')
                pause(2.)
                if a.mode=='small':
                    cmd=[sys.executable,str(tools/'guarded_position_sitl.py'),'--output-prefix',str(out/'motion'),
                         '--segment-distance-m','.4','--open-motion-audit']
                else:
                    route=out/'route.json';route.write_text(json.dumps(route_offsets)+'\n')
                    env['ASTRA_TEST_BOUNDS_RADIUS_M']='9.0'
                    cmd=[sys.executable,str(tools/'ego_sitl_flight.py'),'--output-prefix',str(out/'motion'),
                         '--obstacle-audit','--single-goal','--fixed-enu-ned-transform','--velocity-feedforward',
                         '--freeze-tracking-yaw','--max-tracking-error-m','.60','--route-json',str(route)]
                if a.calibrate_ground_height:
                    cmd.append('--calibrate-ground-height')
                if a.executor_audit and a.mode=='obstacle':cmd.append('--executor-audit')
                if a.mode=='obstacle':cmd.extend(['--execution-executor',a.execution_executor])
                driver=start('motion',cmd)
                if a.stop_map_after_handoff_s is None:
                    result=driver.wait(timeout=flight_timeout)
                else:
                    deadline=time.monotonic()+flight_timeout;handoff=None;injection=None
                    while driver.poll() is None:
                        if time.monotonic()>deadline:
                            raise RuntimeError(f'Flight regression exceeded {flight_timeout} seconds')
                        if handoff is None:
                            for line in (out/'motion.log').read_text().splitlines():
                                if line.startswith('{') and 'ego_control_handoff' in line:
                                    candidate=json.loads(line)
                                    if candidate.get('event')=='ego_control_handoff':handoff=candidate
                        if (handoff and injection is None and
                                time.monotonic_ns()-handoff['wall_ns']>=a.stop_map_after_handoff_s*1e9):
                            injection={'kind':'stop_owned_map_process_group','signal':'SIGINT',
                                       'map_process_group':map_proc.pid,'handoff':handoff,
                                       'command_wall_ns':time.monotonic_ns()}
                            os.killpg(map_proc.pid,signal.SIGINT)
                            (out/'fault_injection.json').write_text(json.dumps(injection,indent=2)+'\n')
                        time.sleep(.05)
                    driver_result=driver.wait();motion=json.loads((out/'motion_events.json').read_text())
                    abort=next((e for e in motion['events'] if e['event']=='test_aborted'),None)
                    hold=next((e for e in motion['events'] if e['event']=='hold_target_frozen'),None)
                    expected_fault=(injection is not None and driver_result==1 and
                        map_proc.poll()==0 and abort is not None and hold is not None and
                        abort['wall_ns']>=injection['command_wall_ns'] and
                        0 <= hold['wall_ns']-injection['command_wall_ns'] <= 500_000_000 and
                        motion.get('disarmed') and not motion.get('flight_complete') and
                        'local map not ready or heartbeat stale' in motion.get('fault',''))
                    regression={'expected_fault_observed':bool(expected_fault),'driver_exit_code':driver_result,
                                'map_exit_code':map_proc.poll(),'fault':motion.get('fault'),
                                'disarmed':motion.get('disarmed')}
                    (out/'fault_regression.json').write_text(json.dumps(regression,indent=2)+'\n')
                    result=0 if expected_fault else 1
                (out/'controller_finished').touch()
            for proc in (monitor,audit,health):proc.wait(timeout=8.)
            if scan_probe is not None:scan_probe.wait(timeout=8.)
    except Exception as exc:
        failure=f'{type(exc).__name__}: {exc}'
        print(failure,flush=True)
    finally:
        (out/'controller_finished').touch()
        for name,proc in processes:
            if name in ('monitor','frame','health') and proc.poll() is None:
                try:proc.wait(timeout=8.)
                except subprocess.TimeoutExpired:pass
        for name,proc in reversed(processes):
            signal_owned_group(proc,signal.SIGINT)
        time.sleep(2.)
        for name,proc in reversed(processes):
            signal_owned_group(proc,signal.SIGTERM)
        time.sleep(1.)
        for name,proc in reversed(processes):
            signal_owned_group(proc,signal.SIGKILL)
            proc.wait(timeout=5.)
        for stream in streams:stream.close()
        unchanged={path:hashlib.sha256(Path(path).read_bytes()).hexdigest()==h for path,h in before.items()}
        execution_unchanged=all(hashlib.sha256((REPO/path).read_bytes()).hexdigest()==h
                                for path,h in execution_hashes.items())
        (out/'baseline_preserved.json').write_text(json.dumps(unchanged,indent=2)+'\n')
        (out/'run_summary.json').write_text(json.dumps({
            'mode':a.mode,'exit_code':result,'failure':failure,
            'original_baseline_preserved':all(unchanged.values()),
            'execution_sources_unchanged':execution_unchanged},indent=2)+'\n')
    print(f'Run saved: {out}; result={result}',flush=True)
    raise SystemExit(result)


if __name__=='__main__':main()
