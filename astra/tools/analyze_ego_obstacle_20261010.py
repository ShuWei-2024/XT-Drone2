"""Audit an isolated EGO/PX4 run against the actual static Gazebo boxes."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
import sys
from obstacle_world_geometry_20261010 import clearances as all_clearances, load as load_obstacles
sys.path.append("/home/cyf/dev/px4/venv/lib/python3.12/site-packages")
from pyulog import ULog
p=argparse.ArgumentParser();p.add_argument('run_dir');a=p.parse_args();d=Path(a.run_dir)
r=json.loads((d/'motion_events.json').read_text());events=r['events'];samples=r.get('bridge_samples',[])
s={k:r.get(k) for k in ('flight_complete','disarmed','fault','actual_displacement_enu','endpoint_px4_error_m')}
goals=[x for x in events if x['event']=='ego_goal_sent']
s['goals_livo_m']=[x['goal'] for x in goals]
s['route']=('multiple destination goals; each leg autonomously EGO-planned' if len(goals)>1
            else 'single EGO goal, no manually supplied detour')
s['actual_sampled_path_length_m']=r.get('actual_sampled_path_length_m')
s['frame_alignment_certified']=False
s['test_limits']={'planner_speed_mps':.2,'planner_acceleration_mps2':.25,'tracking_error_m':.6,'truth_clearance_stop_m':.55,'map_inflation_m':.65,'self_return_filter_m':.45,'odom_receipt_age_sec':.25,'map_cloud_age_sec':1.,'odom_step_m':.12}
s['test_limits'].update(r.get('test_configuration', {}))
s['alignment_rotation_source']=r.get('alignment',{}).get('rotation_source','legacy run: see command record')
start=next((x for x in events if x['event']=='ego_control_handoff'),None)
end=next((x for x in events if x['event']=='ego_trajectory_sampled_to_end'),None)
box=np.array([[3.65,4.35,1.65,2.35],[3.65,4.35,-2.35,-1.65],[6.5,7.5,-.4,.4]])
obstacles = load_obstacles(d/'obstacle_geometry.json') if (d/'obstacle_geometry.json').exists() else None
if obstacles:
 s['audited_obstacle_count'] = len(obstacles)
def clearance(xy):
 dx=np.maximum(np.maximum(box[:,0]-xy[:,None,0],xy[:,None,0]-box[:,1]),0.)
 dy=np.maximum(np.maximum(box[:,2]-xy[:,None,1],xy[:,None,1]-box[:,3]),0.)
 return np.min(np.hypot(dx,dy),axis=1)
if samples:
 pos=np.array([x['truth'] for x in samples]);track=[x for x in samples if x['state']=='TRACK_SITL']
 s['bridge_samples']=len(samples);s['tracking_samples']=len(track)
 s['execution_elapsed_sec']=(samples[-1]['sim_ns']-samples[0]['sim_ns'])/1e9
 s['last_ego_execution_truth_m']=samples[-1]['truth'];s['minimum_sampled_center_box_clearance_m']=float(clearance(pos[:,:2]).min())
 if obstacles:s['minimum_sampled_all_obstacle_clearance_m']=float(all_clearances(pos[:,:2],obstacles).min())
 if track:
  err=np.linalg.norm(np.array([x['target'] for x in track])-np.array([x['px4'] for x in track]),axis=1)
  s['px4_tracking_error_p95_m']=float(np.percentile(err,95));s['px4_tracking_error_max_m']=float(err.max())
  s['bridge_send_gap_max_ms']=float(np.diff([x['wall_ns'] for x in track]).max()/1e6)
peaks=[]
for msg in r.get('bsplines',[]):
 t=np.asarray(msg['knots']);k=msg['order'];b=BSpline(t,msg['controls'],k);grid=np.linspace(t[k],t[-k-1],max(2,int((t[-k-1]-t[k])/.005)+1))
 peaks.append([float(np.linalg.norm(b.derivative(i)(grid),axis=1).max()) for i in (1,2)])
s['bspline_count']=len(peaks)
if peaks:s['planned_peak_speed_mps'],s['planned_peak_acceleration_mps2']=np.max(peaks,axis=0).tolist()
rows=list(csv.DictReader(open(d/'frame_samples.csv')))
truth_rows=sorted([x for x in rows if x['source']=='gazebo'],key=lambda x:int(x['measurement_time_ns']))
truth_stamps=np.array([int(x['measurement_time_ns']) for x in truth_rows])
capture_end=goals[0]['sim_ns'] if goals else 0
pairs=[]
for x in rows:
 if x['source']!='fast_livo2' or not capture_end-2_000_000_000<=int(x['measurement_time_ns'])<=capture_end:continue
 i=int(np.argmin(abs(truth_stamps-int(x['measurement_time_ns']))))
 if abs(truth_stamps[i]-int(x['measurement_time_ns']))<=20_000_000:pairs.append((x,truth_rows[i]))
truth_goal=None
if pairs:
 angles=np.array([float(b['yaw_rad'])-float(a['yaw_rad']) for a,b in pairs]);angle=np.arctan2(np.sin(angles).mean(),np.cos(angles).mean())
 rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
 translation=np.median([np.array([float(b[k]) for k in ('x_m','y_m')])-rotation@np.array([float(a[k]) for k in ('x_m','y_m')]) for a,b in pairs],axis=0)
 z_shift=float(np.median([float(b['z_m'])-float(a['z_m']) for a,b in pairs]))
 truth_goal=rotation@np.array(r['goal_livo'][:2])+translation
 s['goal_gazebo_from_initial_alignment_m']=[*truth_goal.tolist(),r['goal_livo'][2]+z_shift]
 s['livo_to_truth_initial_alignment_pairs']=len(pairs)
 if r.get('truth_end') is not None:s['truth_xy_endpoint_error_after_hold_m']=float(np.linalg.norm(np.array(r['truth_end'][:2])-truth_goal))
if start:
 limit=end['sim_ns'] if end else samples[-1]['sim_ns'] if samples else start['sim_ns']
 truth=[x for x in rows if x['source']=='gazebo' and start['sim_ns']<=int(x['measurement_time_ns'])<=limit]
 xyz=np.array([[float(x[k]) for k in ('x_m','y_m','z_m')] for x in truth]); tt=np.array([int(x['measurement_time_ns']) for x in truth])/1e9
 if len(xyz)>1:
  keep=np.r_[True,np.diff(tt)>0];xyz=xyz[keep];tt=tt[keep];v=np.diff(xyz,axis=0)/np.diff(tt)[:,None]
  s['gazebo_truth_samples']=len(xyz);s['minimum_center_box_clearance_m']=float(clearance(xyz[:,:2]).min());s['actual_peak_speed_mps']=float(np.linalg.norm(v,axis=1).max());s['actual_altitude_range_m']=[float(xyz[:,2].min()),float(xyz[:,2].max())]
  if obstacles:s['minimum_all_obstacle_clearance_m']=float(all_clearances(xyz[:,:2],obstacles).min())
  target=truth_goal if truth_goal is not None else np.array(r['goal_livo'][:2]);s['truth_xy_at_ego_end_m']=xyz[-1,:2].tolist()
  s['actual_path_length_m']=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
  if truth_goal is not None and len(goals)==1:
   line=np.linspace(xyz[0,:2],truth_goal,1001)
   s['direct_start_goal_minimum_center_box_clearance_m']=float(clearance(line).min())
   s['truth_xy_endpoint_error_m']=float(np.linalg.norm(xyz[-1,:2]-truth_goal))
  import matplotlib;matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  from matplotlib.patches import Rectangle, Circle
  fig,ax=plt.subplots(figsize=(11,5));ax.plot(xyz[:,0],xyz[:,1],label='Gazebo actual path')
  ax.plot([xyz[0,0],target[0]],[xyz[0,1],target[1]],'--',c='gray',label='Direct start to goal (initial frame alignment)')
  ax.scatter(*target,c='purple',marker='*',s=90,label='Requested goal')
  if len(goals)>1 and truth_goal is not None:
   # Each goal is transformed with the preflight calibration, never refitted.
   ax.lines[-1].remove();ax.collections[-1].remove()
   truth_goals=[rotation@np.array(g['goal'][:2])+translation for g in goals]
   points=np.vstack([xyz[0,:2],truth_goals])
   ax.plot(points[:,0],points[:,1],'--',c='gray',label='Direct segments between requested goals')
   unique_goals={}
   for i,point in enumerate(truth_goals):
    unique_goals.setdefault(tuple(np.round(point,6)),[]).append(i+1)
   for i,(point,indices) in enumerate(unique_goals.items()):
    ax.scatter(*point,c='purple',marker='*',s=90,label='Requested goals' if i==0 else None)
    ax.annotate('Goal '+','.join(map(str,indices)),point,xytext=(5,8),textcoords='offset points')
   s['goals_gazebo_from_initial_alignment_m']=[g.tolist() for g in truth_goals]
   if obstacles:
    s['direct_requested_segments_clearance_m']=[float(all_clearances(
     np.linspace(points[i],points[i+1],1001),obstacles).min()) for i in range(len(goals))]
  for i,(xmin,xmax,ymin,ymax) in enumerate(box):
   ax.add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,color='tomato',label='Gazebo box' if i==0 else None))
   ax.add_patch(Rectangle((xmin-.65,ymin-.65),xmax-xmin+1.3,ymax-ymin+1.3,fill=False,linestyle=':',edgecolor='tomato',label='0.65 m margin (bounding rectangle)' if i==0 else None))
  if obstacles:
   for item in obstacles:
    if not item['name'].startswith('geometry_rich_'):continue
    x,y=item['center'][:2]
    if item['kind']=='box':
     sx,sy=item['size'][:2];shape=Rectangle((x-sx/2,y-sy/2),sx,sy,color='steelblue',alpha=.7)
    else:shape=Circle((x,y),item['radius'],color='steelblue',alpha=.7)
    ax.add_patch(shape)
  ax.scatter(*xyz[0,:2],c='green',label='EGO handoff');ax.scatter(*xyz[-1,:2],c='blue',label='EGO end');ax.set_aspect('equal');ax.grid();ax.legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1));ax.set(xlabel='Gazebo X (m)',ylabel='Gazebo Y (m)',title='EGO-Planner → PX4: actual obstacle flight');fig.tight_layout();fig.savefig(d/'obstacle_actual_path.png',dpi=160);plt.close(fig)
ulog_path=next((d/'log').rglob('*.ulg'),None)
if ulog_path:
 u=ULog(str(ulog_path));data={x.name:x.data for x in u.data_list}
 if 'vehicle_status' in data:
  x=data['vehicle_status'];s['armed_failsafe_samples']=int(np.sum((x['arming_state']==2)&x['failsafe'].astype(bool)))
 if 'estimator_status_flags' in data:
  x=data['estimator_status_flags'];s['external_vision_active_samples']={k:int(np.count_nonzero(x[k])) for k in ('cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw') if k in x}
 if start and 'vehicle_local_position' in data:
  x=data['vehicle_local_position'];last=end['sim_ns'] if end else samples[-1]['sim_ns'] if samples else start['sim_ns'];mask=(x['timestamp']>=start['sim_ns']/1000)&(x['timestamp']<=last/1000)
  s['execution_estimator_resets']={k:int(np.count_nonzero(np.diff(x[k][mask].astype(int)))) for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter')}
 if 'trajectory_setpoint' in data:
  x=data['trajectory_setpoint'];confirmed=0
  for row in samples:
   if row['state']!='TRACK_SITL':continue
   i=int(np.argmin(abs(x['timestamp'].astype(float)-row['sim_ns']/1000)));value=[x[f'position[{j}]'][i] for j in range(3)]
   confirmed+=abs(float(x['timestamp'][i])-row['sim_ns']/1000)<=20000 and np.linalg.norm(np.array(value)-row['target'])<1e-5
  s['ulog_confirms_planner_setpoints']=int(confirmed)
 for topic in ('vehicle_gps_position','sensor_gps','sensor_mag','battery_status'):
  if topic in data and len(data[topic]['timestamp'])>1:s[topic+'_max_gap_sec']=float(np.diff(data[topic]['timestamp'].astype(float)).max()/1e6)
 if start:
  s['sensor_log_execution_coverage']={}
  for topic in ('vehicle_gps_position','sensor_mag','battery_status'):
   if topic not in data:continue
   stamps=data[topic]['timestamp'].astype(float)/1e6
   lo=start['sim_ns']/1e9;hi=(end['sim_ns'] if end else samples[-1]['sim_ns'])/1e9
   active=stamps[(stamps>=lo)&(stamps<=hi)]
   if len(active)>1:s['sensor_log_execution_coverage'][topic]={
    'samples':len(active),'logged_max_gap_sec':float(np.diff(active).max()),
    'first_after_handoff_sec':float(active[0]-lo),'last_before_end_sec':float(hi-active[-1])}
if (d/'cloud_audit.json').exists():
 audit=json.loads((d/'cloud_audit.json').read_text())
 for kind in ('full','rgb'):
  x=[v for v in audit if v['kind']==kind]
  if x:s[kind+'_cloud_point_count_min_median_max']=np.percentile([v['count'] for v in x],[0,50,100]).tolist()
 x=[v for v in audit if v['kind']=='diagnostics']
 if x:s['local_map_voxel_count_min_max']=[min(int(v['voxel_count']) for v in x),max(int(v['voxel_count']) for v in x)]
 if start:
  x=[v for v in x if start['wall_ns']<=v['wall_ns']<=(end or events[-1])['wall_ns']]
  s['map_diagnostic_reasons_during_execution']=sorted(set(v['reason'] for v in x))
summary=json.loads((d/'receipt_summary.json').read_text());s['odom_measurement_age_p95_ms']=summary['odom_clock_age_ms']['p95'];s['real_time_factor']=summary['real_time_factor']
import hashlib
s['original_parameters_unchanged']=all(hashlib.sha256(Path(line.split(maxsplit=1)[1]).read_bytes()).hexdigest()==line.split()[0] for line in (d/'original_parameters.sha256').read_text().splitlines())
(d/'obstacle_execution_summary.json').write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s,indent=2))
