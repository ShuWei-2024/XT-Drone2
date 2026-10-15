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
from sitl_cpu_isolation import cpu_plan, command_for_role
from outdoor_envelope import outdoor_envelope

REPO=Path(__file__).resolve().parents[2]
PX4=Path(os.environ.get('ASTRA_PX4_DIR', str(Path.home()/'dev/px4/PX4-Autopilot-1.17')))


def load_route(path, maximum_radius_m=7.):
    route=json.loads(path.read_text())
    if (not isinstance(route,list) or not 1 <= len(route) <= 8 or
        any(not isinstance(xy,list) or len(xy)!=2 or
            not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                    for v in xy) or math.hypot(*xy)>maximum_radius_m for xy in route)):
        raise ValueError(f'Route must contain 1..8 finite XY offsets within {maximum_radius_m:g} m of the frozen origin')
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


def wait_owned_groups(processes, seconds):
    """Allow owned DDS participants to close before escalating shutdown."""
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        alive=False
        for _,proc in processes:
            proc.poll()
            try:os.killpg(proc.pid,0);alive=True
            except ProcessLookupError:pass
        if not alive:return
        time.sleep(.1)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['timer','static','small','obstacle'])
    p.add_argument('name');p.add_argument('--seconds',type=float,default=60.)
    p.add_argument('--world-preset',choices=['geometry-rich','indoor-hall','replica-office0','replica-scan','baylands'],default='geometry-rich',
                   help='Replica supports static checks or the prepared guarded indoor obstacle route')
    p.add_argument('--scan-dir',type=Path,help='Prepared real scan directory; requires replica-scan preset')
    p.add_argument('--calibrate-ground-height',action='store_true')
    p.add_argument('--external-vision', action='store_true',
                   help='Isolated position-only FAST-LIVO2 EV experiment; XY/Z fusion, vision height reference')
    p.add_argument('--lio-only-diagnostic',action='store_true',
                   help='Isolated comparison: disable image updates in the copied estimator config')
    p.add_argument('--lio-voxel-size-m',type=float,choices=[.25,.5],default=.5,
                   help='Isolated plane-map resolution comparison; default retains 0.5 m')
    p.add_argument('--plane-quality-diagnostic',action='store_true',
                   help='Copied config only: reject thick or line-like plane fits; subdivide rejected voxels')
    p.add_argument('--capture-scan-inputs',action='store_true',
                   help='Sparse raw cloud and IMU/pose capture for offline sensor geometry audit')
    p.add_argument('--executor-audit',action='store_true',help='Trace flight callbacks and DDS queue timestamps')
    p.add_argument('--isolate-estimator-cpus',action='store_true',
                   help='Reserve two physical CPU cores for FAST-LIVO2, pin owned simulation processes separately')
    p.add_argument('--endpoint-hold-seconds',type=float,default=3.,
                   help='Obstacle mode: guarded final hover for SITL recording, 3..300 seconds')
    p.add_argument('--execution-executor',choices=['ready-batch','events'],
                   help='Whole-map obstacle SITL defaults to events; other scenes use ready-batch. Explicit choice overrides the default')
    p.add_argument('--route-json',type=Path,
                   help='Obstacle test goal offsets; up to 8 goals, each bounded to 7 m')
    p.add_argument('--rviz-goal',action='store_true',
                   help='Prepared scan: open RViz and mission controls, edit waypoints and explicitly start')
    p.add_argument('--end-action', choices=('land','return_home'), default='land',
                   help='Noninteractive mission completion; EGO return then official PX4 land')
    p.add_argument('--rviz-goal-timeout-sec',type=float,default=0.,
                   help='Ground editing timeout; 0 waits until explicit Start or close')
    p.add_argument('--stop-map-after-handoff-s',type=float,
                   help='SITL fault regression: stop this run\'s map after planner control handoff')
    p.add_argument('--stop-ev-after-handoff-s',type=float,
                   help='SITL fault regression: stop this run\'s EV publisher after planner handoff')
    p.add_argument('--baro-noise-m',type=float,choices=[.1,.3],default=.3,
                   help='Isolated SITL EKF baro sigma: default 0.3 m matches the model\'s '
                        '3 Pa pressure noise (about 0.25 m); 0.1 m retains the old comparison')
    p.add_argument('--gps-delay-ms',type=float,choices=[0.,110.],default=110.,
                   help='Isolated single-factor GPS delay comparison; default preserves 110 ms')
    p.add_argument('--pressure-noise-pa',type=float,choices=[0.,.05,3.],default=3.,
                   help='Diagnostic noise ablation only; normal simulation retains 3 Pa. '
                        '0 Pa can trigger PX4 stuck-sensor detection; use 0.05 Pa for low-noise diagnosis')
    a=p.parse_args()
    if a.external_vision and (a.mode == 'timer' or not a.calibrate_ground_height):
        p.error('EV experiment requires measured ground-height calibration and static/small/obstacle mode')
    shm=os.statvfs('/dev/shm')
    shm_free=shm.f_bavail*shm.f_frsize
    if shm_free < 512*1024**2:
        p.error(f'DDS shared memory has only {shm_free/1024**2:.1f} MiB free; '
                'need at least 512 MiB before starting SITL. Inspect unused DDS caches; '
                'do not delete files used by running ROS processes.')
    affinity_plan=cpu_plan() if a.isolate_estimator_cpus else dict(enabled=False,reason='not requested')
    if (not math.isfinite(a.endpoint_hold_seconds) or not 3. <= a.endpoint_hold_seconds <= 300. or
            (a.mode!='obstacle' and a.endpoint_hold_seconds!=3.)):
        p.error('Extended final hover requires obstacle mode and a finite duration in [3,300] seconds')
    if a.execution_executor=='events' and a.mode!='obstacle':
        p.error('Event executor candidate is restricted to isolated obstacle SITL')
    is_outdoor=a.world_preset=='baylands'
    is_scan=a.world_preset in ('replica-office0','replica-scan','baylands')
    if a.rviz_goal and (not is_scan or a.mode!='obstacle'):
        p.error('RViz goal flight requires obstacle mode in a prepared scan')
    if not math.isfinite(a.rviz_goal_timeout_sec) or (a.rviz_goal_timeout_sec!=0. and not 30.<=a.rviz_goal_timeout_sec<=600.):
        p.error('RViz goal timeout must be zero (no ground timeout) or in [30,600] seconds')
    if (a.world_preset in ('replica-scan','baylands')) != (a.scan_dir is not None):
        p.error('replica-scan and baylands require --scan-dir; other presets do not accept it')
    replica_dir=a.scan_dir.resolve() if a.scan_dir else REPO/'astra/runtime/maps/replica_office0'
    if a.rviz_goal and a.route_json is None:
        a.route_json=replica_dir/'route.json'
    scan_model_name='replica_office0'
    if is_scan:
        if a.mode not in ('static','obstacle'):
            p.error('Replica office0 requires static mode or its prepared mesh-guarded obstacle route')
        if a.mode=='obstacle' and not a.route_json:
            p.error('Replica flight requires an explicit prepared indoor route')
        if not (replica_dir/'manifest.json').is_file():
            p.error('Prepare the scan first with astra/tools/prepare_replica_office0.py')
        manifest=json.loads((replica_dir/'manifest.json').read_text())
        scan_model_name=manifest.get('model_name','replica_office0')
        if not re.fullmatch('replica_(office[0-4]|room[02])|astra_baylands',scan_model_name):
            p.error('Unexpected scan model name')
        for asset,digest in manifest['asset_sha256'].items():
            if hashlib.sha256((replica_dir/scan_model_name/'meshes'/asset).read_bytes()).hexdigest()!=digest:
                p.error('Converted Replica asset changed: '+asset)
    if a.route_json and a.mode!='obstacle':p.error('Custom routes require obstacle mode')
    outdoor_config=None
    try:
        if is_outdoor and a.mode=='obstacle':
            import numpy as np
            flight_precheck=json.loads((replica_dir/'flight_precheck.json').read_text())
            with np.load(replica_dir/'mesh_guard.npz',allow_pickle=False) as guard:
                outdoor_config=outdoor_envelope(flight_precheck,json.loads(str(guard['metadata'])))
        radius=outdoor_config['goal_radius_m'] if outdoor_config else 7.
        route_offsets=load_route(a.route_json,radius) if a.route_json else [[5,3],[0,0]]
    except (OSError,ValueError) as exc:p.error(str(exc))
    if is_scan and a.mode=='obstacle':
        try:
            flight_precheck=json.loads((replica_dir/'flight_precheck.json').read_text())
            if (route_offsets!=flight_precheck['route_offsets_m'] or
                flight_precheck['collision_sha256']!=manifest['asset_sha256']['collision.stl'] or
                hashlib.sha256((replica_dir/'world.sdf').read_bytes()).hexdigest()!=flight_precheck['world_sha256'] or
                hashlib.sha256((replica_dir/scan_model_name/'model.sdf').read_bytes()).hexdigest()!=flight_precheck['model_sdf_sha256'] or
                hashlib.sha256((replica_dir/'mesh_guard.npz').read_bytes()).hexdigest()!=flight_precheck['guard_sha256']):
                p.error('Prepared route or guard differs from the flight precheck')
        except (OSError,ValueError) as exc:p.error('Prepare scan flight first: '+str(exc))
    if a.execution_executor is None:
        a.execution_executor=('events' if is_outdoor and a.mode=='obstacle' and
                              flight_precheck.get('whole_map_selection',False) else 'ready-batch')
    if a.execution_executor=='events':
        # Check the optional installed ROS capability before any simulator or
        # arming. Do not discover a missing executor after takeoff, or silently
        # fall back to the receiver that timed out on this prepared scene.
        try:
            from rclpy.experimental.events_executor import EventsExecutor
            from rclpy.impl.implementation_singleton import rclpy_implementation
            if not hasattr(rclpy_implementation,'EventsExecutor'):
                raise ImportError('rclpy C++ EventsExecutor unavailable')
        except ImportError as exc:
            p.error('Event executor unavailable; update ROS 2 Jazzy or explicitly select --execution-executor ready-batch: '+str(exc))
    takeoff_height_m=flight_precheck.get('takeoff_height_m',1.7) if is_scan and a.mode=='obstacle' else 1.7
    if not isinstance(takeoff_height_m,(int,float)) or isinstance(takeoff_height_m,bool) or not math.isfinite(takeoff_height_m) or not .8<=takeoff_height_m<=1.7:
        p.error('Prepared takeoff height must be finite and in [0.8,1.7] metres')
    alignment_height_m=flight_precheck.get('alignment_height_m',takeoff_height_m) if is_scan and a.mode=='obstacle' else takeoff_height_m
    if not isinstance(alignment_height_m,(int,float)) or isinstance(alignment_height_m,bool) or not math.isfinite(alignment_height_m) or not takeoff_height_m<=alignment_height_m<=1.7:
        p.error('Prepared alignment height must be between cruise height and 1.7 m')
    planner_ceiling_m=flight_precheck.get('planner_virtual_ceiling_m',2.3) if is_scan and a.mode=='obstacle' else 2.3
    if not isinstance(planner_ceiling_m,(int,float)) or isinstance(planner_ceiling_m,bool) or not math.isfinite(planner_ceiling_m) or not takeoff_height_m+.3<=planner_ceiling_m<=2.3:
        p.error('Prepared planner ceiling must leave at least 0.3 m over the takeoff height and not exceed 2.3 m')
    optimization_clearance_m=flight_precheck.get('optimization_clearance_m',.8) if is_scan and a.mode=='obstacle' else .8
    if not isinstance(optimization_clearance_m,(int,float)) or isinstance(optimization_clearance_m,bool) or not math.isfinite(optimization_clearance_m) or not .2<=optimization_clearance_m<=.8:
        p.error('Optimizer clearance over the inflated sensor map must be finite and in [0.2,0.8] metres')
    map_retention_sec=flight_precheck.get('static_obstacle_retention_sec',3.) if is_scan and a.mode=='obstacle' else 3.
    if not isinstance(map_retention_sec,(int,float)) or isinstance(map_retention_sec,bool) or not math.isfinite(map_retention_sec) or not 3.<=map_retention_sec<=180.:
        p.error('Static scan obstacle retention must be finite and in [3,180] seconds')
    static_columns=flight_precheck.get('static_descent_columns',False) if is_scan and a.mode=='obstacle' else False
    if not isinstance(static_columns,bool):p.error('Static descent columns must be a boolean')
    planner_leg_timeout_s=flight_precheck.get('planner_leg_timeout_s',100.) if is_scan and a.mode=='obstacle' else 100.
    if (type(planner_leg_timeout_s) not in (int,float) or not math.isfinite(planner_leg_timeout_s)
            or not 100.<=planner_leg_timeout_s<=500.):p.error('Prepared planner leg timeout must be in [100,500] seconds')
    flight_timeout=(max(330.,150.+planner_leg_timeout_s*(len(route_offsets)+(a.end_action=='return_home')))+a.endpoint_hold_seconds-3.
                    if a.mode=='obstacle' else 330.)
    if a.rviz_goal:flight_timeout=150.+9.*(planner_leg_timeout_s+120.)
    if a.stop_map_after_handoff_s is not None and (a.mode!='obstacle' or
            not 1. <= a.stop_map_after_handoff_s <= 10.):
        p.error('Map-stop regression requires obstacle mode and a delay of 1..10 seconds')
    if a.stop_ev_after_handoff_s is not None and (a.mode!='obstacle' or not a.external_vision or
            not 1. <= a.stop_ev_after_handoff_s <= 10.):
        p.error('EV-stop regression requires EV obstacle mode and a delay of 1..10 seconds')
    if a.stop_ev_after_handoff_s is not None and a.stop_map_after_handoff_s is not None:
        p.error('Inject one process failure at a time')
    injection_delay=(a.stop_ev_after_handoff_s if a.stop_ev_after_handoff_s is not None
                     else a.stop_map_after_handoff_s)
    if not re.fullmatch('[a-zA-Z0-9_-]{1,64}',a.name):p.error('Use a new short run name')
    if os.environ.get('ASTRA_117_ENV_LOADED')!='1':p.error('Use astra/run_117.bash to load the isolated ROS environment')
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=PX4,text=True).strip() != json.loads((REPO/'astra/dependencies.lock.json').read_text())['px4']['commit']:
        p.error('PX4 commit differs from the lock file')
    for patch in ('astra/baseline/px4_1_15_3_hrt.patch','astra/config/px4_117_gz_magnetometer.patch',
                  'astra/config/px4_117_dds_local_position.patch'):
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
    source_paths += list((REPO/'astra/packages/ego_planner_jazzy').rglob('*.cpp'))
    source_paths += list((REPO/'astra/packages/ego_planner_jazzy').rglob('*.h'))
    if a.rviz_goal:source_paths.append(REPO/'astra/config'/
        ('rviz_outdoor_flight.rviz' if is_outdoor else 'rviz_goal_flight.rviz'))
    execution_hashes={}
    for source in source_paths:
        relative=source.relative_to(REPO);dest=snapshot/relative
        dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        execution_hashes[str(relative)]=hashlib.sha256(source.read_bytes()).hexdigest()
    for relative in ('install_117/astra_fast_livo2_core/lib/astra_fast_livo2_core/fastlivo_mapping',
                     'install_117/plan_env/lib/libplan_env.so',
                     'install_117/path_searching/lib/libpath_searching.so',
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
    if is_scan:
        shutil.copytree(replica_dir/scan_model_name,models/scan_model_name)
        scan_sdf=models/scan_model_name/'model.sdf';scan_tree=ET.parse(scan_sdf)
        for texture in scan_tree.getroot().iter('albedo_map'):
            texture.text=str(models/scan_model_name/texture.text)
        scan_tree.write(scan_sdf,encoding='utf-8',xml_declaration=True)
        shutil.copy2(replica_dir/'manifest.json',out/'map_manifest.json')
        license_file='LICENSE.txt' if is_outdoor else 'REPLICA_LICENSE.txt'
        shutil.copy2(replica_dir/license_file,out/license_file)
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
    source_world=(replica_dir/'world.sdf' if is_scan else
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
    if a.external_vision:
        # Odom already references base_link. Do not apply the LiDAR/IMU lever arm twice.
        overrides.update(EKF2_EV_CTRL=3, EKF2_HGT_REF=3, EKF2_EV_DELAY=0,
                         EKF2_EV_POS_X=0, EKF2_EV_POS_Y=0, EKF2_EV_POS_Z=0,
                         EKF2_EV_NOISE_MD=0, EKF2_EVP_NOISE=.1, SDLOG_PROFILE=129)
        env['ASTRA_REQUIRE_EXTERNAL_VISION']='1'
    else:
        env.pop('ASTRA_REQUIRE_EXTERNAL_VISION', None)
    if outdoor_config and outdoor_config.get('transit_enabled'):
        # Only this copied SITL instance: direct RTL at the certified cruise
        # altitude. PX4 defaults (60/30 m) are inappropriate for this volume.
        overrides.update(RTL_RETURN_ALT=takeoff_height_m, RTL_DESCEND_ALT=takeoff_height_m,
                         RTL_CONE_ANG=0, RTL_MIN_DIST=.5, RTL_LAND_DELAY=0,
                         RTL_TYPE=0, RTL_PLD_MD=0, NAV_ACC_RAD=.2, MPC_XY_CRUISE=.3)
        # Preserve EV as height reference. Selecting GNSS just to initialize
        # altitude made EV height bias adapt to the barometer (GPS height off).
        # Establish the missing absolute origin separately, disarmed and only
        # after EV fusion starts, before the flight supervisor freezes home.
        if not a.external_vision:overrides['EKF2_HGT_REF']=1
    env.update({f'PX4_PARAM_{key}':str(value) for key,value in overrides.items()})
    startup.write_text('. "${R}etc/init.d-posix/rcS"\n')
    planner_speed=.10 if is_scan and not is_outdoor else .20
    planner_acceleration=.10 if is_scan and not is_outdoor else .25
    runtime={'mode':a.mode,'world':str(world),'px4':str(PX4),'namespace':'x500_0',
             'execution_executor':a.execution_executor,
             'external_vision_enabled':a.external_vision,
             'disarmed_gnss_origin_initialization':bool(a.external_vision and outdoor_config and
                                                       outdoor_config.get('transit_enabled')),
             'world_preset':a.world_preset,'static_only_scan':is_scan and a.mode=='static',
             'prepared_outdoor_envelope':outdoor_config,
            'mesh_guard_enabled':is_scan and a.mode=='obstacle',
             'shared_execution_envelope':is_scan and a.mode=='obstacle',
             'shared_envelope_tracking_reserve_m':.4 if is_scan and a.mode=='obstacle' else None,
             'scan_dir':str(replica_dir) if is_scan else None,
             'takeoff_height_m':takeoff_height_m,
             'alignment_height_m':alignment_height_m,
             'planner_virtual_ceiling_m':planner_ceiling_m,
             'optimization_clearance_m':optimization_clearance_m,
             'static_obstacle_retention_sec':map_retention_sec,
             'static_descent_columns':static_columns,
             'descent_column_inflation_m':1.1 if is_outdoor else 0.,
             'column_obstacle_max_z_m':outdoor_config['column_obstacle_max_z_m'] if outdoor_config else planner_ceiling_m,
             'planner_motion_limits':dict(maximum_speed_mps=planner_speed,
                                          maximum_acceleration_mps2=planner_acceleration),
             'lio_only_diagnostic':a.lio_only_diagnostic,
             'lio_voxel_size_m':a.lio_voxel_size_m,
             'plane_quality_diagnostic':a.plane_quality_diagnostic,
             'capture_scan_inputs':a.capture_scan_inputs,
             'executor_audit':a.executor_audit,
             'execution_executor':a.execution_executor,
             'endpoint_hold_seconds':a.endpoint_hold_seconds,
             'rviz_goal_selection':a.rviz_goal,
             'px4_binary_sha256':hashlib.sha256((PX4/'build/px4_sitl_default/bin/px4').read_bytes()).hexdigest(),
             'magnetometer_convention':'Gazebo world ENU / tesla -> body FLU -> PX4 body FRD / gauss',
             'magnetometer_noise_scale_from_official_gauss_model':1.e-4,
             'route_offsets_m':route_offsets if a.mode=='obstacle' else None,
             'flight_timeout_s':flight_timeout,
             'planner_leg_timeout_s':planner_leg_timeout_s,
             'ground_edit_timeout_s':a.rviz_goal_timeout_sec if a.rviz_goal else None,
             'pressure_noise_pa':a.pressure_noise_pa,
             'diagnostic_noiseless_pressure':a.pressure_noise_pa==0.,
             'diagnostic_reduced_pressure_noise':a.pressure_noise_pa!=3.,
             'sensors_depth_removed':True,'parameter_overrides':overrides,'isolated_rootfs':str(out),'ros_domain':94,'gz_partition':env['GZ_PARTITION']}
    runtime['odom_reference'] = 'base_link; internal LIVO state remains at imu_link'
    runtime['px4_local_position_dds_limit_hz'] = 100
    runtime['px4_binary_sha256'] = hashlib.sha256((PX4/'build/px4_sitl_default/bin/px4').read_bytes()).hexdigest()
    runtime['cpu_affinity']=affinity_plan
    runtime['dds_shared_memory_free_at_start_bytes']=shm_free
    runtime['planner_horizon_m']=10. if is_outdoor else 6.
    # Outdoor scans may have no close returns at cruise height. Retain measured
    # geometry over the planner horizon plus inflation/tracking reserve instead
    # of clipping it to the indoor 5 m sphere. Health/capacity gates stay intact.
    runtime['local_map_radius_m']=12. if is_outdoor else 5.
    runtime['local_map_max_voxels']=20000 if is_outdoor else 5000
    if is_outdoor and flight_precheck.get('whole_map_selection',False):
        # Far-side park scans exceeded the old 20k budget at 20,039 cells.
        # Keep radius/TTL, obstacle resolution, freshness and build deadline;
        # allocate a fixed larger budget, never discard arbitrary obstacles.
        runtime['local_map_max_voxels']=50000
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
        cmd=command_for_role(cmd,name,affinity_plan)
        proc=subprocess.Popen(cmd,env=env,cwd=REPO,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,proc));return proc
    def pause(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            for name,proc in processes:
                if name in ('agent','simulator','gcs_heartbeat','livo','map','planner','external_vision') and proc.poll() is not None:
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
                 '-p','use_sim_time:=true','-p','self_filter_radius_m:=0.45',
                 '-p',f'radius_m:={runtime["local_map_radius_m"]}',
                 '-p',f'max_map_voxels:={runtime["local_map_max_voxels"]}',
                 '-p',f'voxel_ttl_sec:={map_retention_sec}',
                 '-p',f'static_descent_columns:={str(static_columns).lower()}',
                 '-p','descent_column_inflation_m:=1.1' if is_outdoor else 'descent_column_inflation_m:=0.0',
                 '-p',f'column_obstacle_max_z_m:={outdoor_config["column_obstacle_max_z_m"] if outdoor_config else planner_ceiling_m}',
                 '-p',f'column_ceiling_m:={planner_ceiling_m}'])
            if a.external_vision:
                ev_proc=start('external_vision',[sys.executable,str(tools/'external_vision_sitl.py'), '--output',str(out)])
            start('planner',['ros2','launch','astra_fast_livo2','ego_planner_guarded.launch.py',
                 f'max_vel:={planner_speed}',
                 f'max_acc:={planner_acceleration}',
                 f'require_execution_envelope:={str(is_scan and a.mode=="obstacle").lower()}',
                 f'map_size_xy_m:={outdoor_config["planner_map_size_xy_m"] if outdoor_config else 32.0}' if is_outdoor else 'map_size_xy_m:=16.0',
                 'planning_horizon:=10.0' if is_outdoor else 'planning_horizon:=6.0','max_tracking_error_m:=0.60',f'virtual_ceiling_m:={planner_ceiling_m}',
                 f'optimization_clearance_m:={optimization_clearance_m}',
                 'min_goal_distance_m:=0.04' if is_scan else 'min_goal_distance_m:=0.20'])
            pause(12.)
            if runtime['disarmed_gnss_origin_initialization']:
                origin=start('global_origin',[sys.executable,str(tools/'initialize_ev_global_origin.py'),
                                              '--output',str(out/'global_origin.json')])
                deadline=time.monotonic()+65.
                while origin.poll() is None and time.monotonic()<deadline:pause(.2)
                if origin.poll()!=0:
                    raise RuntimeError('Disarmed EV/global-origin initialization failed; no arming commands sent')
            if a.rviz_goal:
                if not env.get('DISPLAY'):raise RuntimeError('RViz2 requires a desktop DISPLAY')
                start('rviz_scene',[sys.executable,str(tools/'rviz_scene_display.py'),
                     '--mesh',str(replica_dir/scan_model_name/'meshes'/f'{manifest["scene"]}.dae'),
                     '--stop-file',str(out/'controller_finished'),'--output',str(out/'display_map.json')]+
                     (['--opaque-background'] if is_outdoor else []))
                start('mission_panel',[sys.executable,str(tools/'rviz_mission_panel.py')])
                rviz_config='rviz_outdoor_flight.rviz' if is_outdoor else 'rviz_goal_flight.rviz'
                rviz=start('rviz',['rviz2','-d',str(REPO/'astra/config'/rviz_config),
                                  '--ros-args','-p','use_sim_time:=true'])
            seconds=(float('inf') if a.rviz_goal else
                     a.seconds if a.mode=='static' else flight_timeout+60.)
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
            if is_scan:
                scan_probe=start('scan_probe',[sys.executable,str(tools/'probe_scan_scene.py'),
                     '--seconds',str(seconds),'--output',str(out),
                     '--stop-file',str(out/'controller_finished')]+(['--capture-inputs'] if a.capture_scan_inputs else [])+
                     (['--capture-map'] if a.mode=='obstacle' else []))
            if a.mode=='static':
                pause(seconds+1.);health.wait(timeout=5.)
                result=0 if json.loads((out/'health.json').read_text())['passed'] else 1
                if scan_probe is not None:
                    scan_probe.wait(timeout=8.)
                    if not json.loads((out/'scan_probe.json').read_text())['passed']:result=1
            else:
                precheck=start('preflight_health',[sys.executable,str(tools/'health_probe_117.py'),
                    '--seconds','12','--healthy-timeout-sec','60',
                    '--output',str(out/'preflight_health.json')])
                print('Preflight: waiting up to 60 s for healthy sensors and map; no arming commands sent',flush=True)
                precheck_code=precheck.wait(timeout=68.)
                precheck_result=json.loads((out/'preflight_health.json').read_text())
                if precheck_code != 0 or not precheck_result['passed']:
                    failed=', '.join(key for key,passed in precheck_result['checks'].items() if not passed)
                    raise RuntimeError('Passive health gate failed after bounded wait: '+failed+
                                       '; no arming commands sent; see '+str(out/'preflight_health.json'))
                print('Preflight health passed; opening guarded mission selection',flush=True)
                pause(2.)
                if a.mode=='small':
                    cmd=[sys.executable,str(tools/'guarded_position_sitl.py'),'--output-prefix',str(out/'motion'),
                         '--segment-distance-m','.4','--open-motion-audit']
                else:
                    route=out/'route.json';route.write_text(json.dumps(route_offsets)+'\n')
                    env['ASTRA_TEST_BOUNDS_RADIUS_M']=str(outdoor_config['supervisor_radius_m']) if is_outdoor else '9.0'
                    cmd=[sys.executable,str(tools/'ego_sitl_flight.py'),'--output-prefix',str(out/'motion'),
                         '--obstacle-audit','--single-goal','--fixed-enu-ned-transform','--velocity-feedforward',
                         '--freeze-tracking-yaw','--max-tracking-error-m','.60','--route-json',str(route),
                         '--takeoff-height-m',str(takeoff_height_m),'--alignment-height-m',str(alignment_height_m)]
                if a.calibrate_ground_height:
                    cmd.append('--calibrate-ground-height')
                if a.executor_audit and a.mode=='obstacle':cmd.append('--executor-audit')
                if a.mode=='obstacle':cmd.extend(['--execution-executor',a.execution_executor,
                                               '--planner-leg-timeout-s',str(planner_leg_timeout_s),
                                               '--endpoint-hold-seconds',str(a.endpoint_hold_seconds),
                                               '--end-action',a.end_action])
                if a.rviz_goal:cmd.extend(['--rviz-mission','--rviz-goal-timeout-sec',str(a.rviz_goal_timeout_sec)])
                if is_outdoor:cmd.extend(['--maximum-route-radius-m',str(outdoor_config['goal_radius_m'])])
                driver=start('motion',cmd)
                if a.rviz_goal and injection_delay is None:
                    started_file=out/'motion_mission_started'
                    mission_deadline=None
                    print('等待选点：添加航点后请在中文控制窗口点击“开始任务”。'+
                          ('地面等待不限时；Ctrl+C 可退出。' if not a.rviz_goal_timeout_sec else
                           f'地面等待限时 {a.rviz_goal_timeout_sec:g} 秒。'),flush=True)
                    while driver.poll() is None:
                        if started_file.exists() and mission_deadline is None:
                            mission_deadline=time.monotonic()+flight_timeout
                            print('收到开始任务；飞行时限和保护已启用。',flush=True)
                        if mission_deadline is not None and time.monotonic()>mission_deadline:
                            raise RuntimeError(f'Flight regression exceeded {flight_timeout} seconds after START')
                        pause(.2)
                    result=driver.returncode
                elif injection_delay is None:
                    result=driver.wait(timeout=flight_timeout)
                else:
                    stopped_proc=ev_proc if a.stop_ev_after_handoff_s is not None else map_proc
                    stopped_name='external_vision' if a.stop_ev_after_handoff_s is not None else 'map'
                    expected_reason=('External Vision fusion unavailable or heartbeat stale'
                        if a.stop_ev_after_handoff_s is not None else 'local map not ready or heartbeat stale')
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
                                time.monotonic_ns()-handoff['wall_ns']>=injection_delay*1e9):
                            injection={'kind':f'stop_owned_{stopped_name}_process_group','signal':'SIGINT',
                                       f'{stopped_name}_process_group':stopped_proc.pid,'handoff':handoff,
                                       'command_wall_ns':time.monotonic_ns()}
                            os.killpg(stopped_proc.pid,signal.SIGINT)
                            (out/'fault_injection.json').write_text(json.dumps(injection,indent=2)+'\n')
                        time.sleep(.05)
                    driver_result=driver.wait();motion=json.loads((out/'motion_events.json').read_text())
                    abort=next((e for e in motion['events'] if e['event']=='test_aborted'),None)
                    hold=next((e for e in motion['events'] if e['event']=='hold_target_frozen'),None)
                    expected_fault=(injection is not None and driver_result==1 and
                        stopped_proc.poll()==0 and abort is not None and hold is not None and
                        abort['wall_ns']>=injection['command_wall_ns'] and
                        0 <= hold['wall_ns']-injection['command_wall_ns'] <= 500_000_000 and
                        motion.get('disarmed') and not motion.get('flight_complete') and
                        expected_reason in motion.get('fault',''))
                    regression={'expected_fault_observed':bool(expected_fault),'driver_exit_code':driver_result,
                                'map_exit_code':map_proc.poll(),'fault':motion.get('fault'),
                                'stopped_process':stopped_name,'stopped_process_exit_code':stopped_proc.poll(),
                                'hold_after_injection_ms':(hold['wall_ns']-injection['command_wall_ns'])/1e6 if hold and injection else None,
                                'disarmed':motion.get('disarmed')}
                    (out/'fault_regression.json').write_text(json.dumps(regression,indent=2)+'\n')
                    result=0 if expected_fault else 1
                (out/'controller_finished').touch()
                if a.rviz_goal:
                    motion_result=json.loads((out/'motion_events.json').read_text())
                    if motion_result.get('fault'):
                        failure=motion_result['fault']
                        print('任务结束原因：'+failure,flush=True)
                    runtime['rviz_selected_goal']=motion_result.get('rviz_selected_goal')
                    runtime['route_offsets_m']=motion_result.get('route_offsets_m',[])
                    (out/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
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
        wait_owned_groups(processes,8.)
        for name,proc in reversed(processes):
            signal_owned_group(proc,signal.SIGTERM)
        wait_owned_groups(processes,3.)
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
