"""Guard GNSS origin initialization independently from the EV local height datum."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from initialize_ev_global_origin import (origin_candidate, absolute_origin_valid,
    check_origin_displacement, position_stability_window)
import initialize_ev_global_origin as origin_module


def inputs():
    return [NS(x=0.,y=0.,z=.03,vx=.01,vy=.02,vz=0.,ref_lat=47.4,ref_lon=8.5,
               xy_valid=True,z_valid=True,xy_global=True,timestamp_sample=990_000),
            NS(altitude_msl_m=488.,eph=.5,epv=.8,vel_m_s=.03,fix_type=3,
               satellites_used=12,vel_ned_valid=True,timestamp_sample=900_000),
            NS(arming_state=1),NS(cs_ev_pos=True,cs_ev_hgt=True),1_000_000]


def test_absolute_datum_preserves_existing_xy_origin_and_local_z():
    values=inputs()
    assert origin_candidate(*values)==[47.4,8.5,488.03]
    assert values[0].z==.03


def test_local_global_flag_and_zero_altitude_do_not_prove_absolute_origin():
    local=NS(z_global=True,ref_alt=0.)
    assert not absolute_origin_valid(local,NS(alt_valid=False))
    assert not absolute_origin_valid(local,None)
    assert absolute_origin_valid(local,NS(alt_valid=True))


def test_origin_displacement_failure_retains_exact_three_dimensional_sample():
    local=NS(x=.02,y=.02,z=.02,timestamp_sample=123_456,
        xy_reset_counter=6,z_reset_counter=3,heading_reset_counter=1,ref_alt=.2921)
    report={}
    with pytest.raises(RuntimeError,match='more than 0.03 m'):
        check_origin_displacement([0.,0.,0.],local,report)
    failure=report['local_displacement_failure']
    assert failure['displacement_m']==pytest.approx(3**.5*.02)
    assert failure['after_local_ned_m']==[.02,.02,.02]
    assert failure['timestamp_sample_us']==123_456
    assert failure['after_resets']==[6,3,1]


def test_origin_displacement_at_existing_boundary_still_passes():
    report={}
    check_origin_displacement([0.,0.,0.],NS(x=.03,y=0.,z=0.),report)
    assert not report


def test_position_stability_window_accepts_small_reset_free_motion():
    samples=[(i*100_000, .001*(i%3), -.001*(i%2), .002*(i%2), (1,2,0))
             for i in range(21)]
    stable,details=position_stability_window(samples)
    assert stable
    assert details['duration_us']==2_000_000
    assert max(details['axis_excursion_m']) < .01


def test_position_stability_window_accepts_sampled_1992ms_coverage_for_2s_window():
    samples=[(i*8_000,0.,0.,0.,(1,2,0)) for i in range(250)]
    stable,details=position_stability_window(samples)
    assert stable
    assert details['duration_us']==1_992_000


def test_position_stability_window_rejects_known_scale_slow_xy_drift():
    # The failed Baylands origin run drifted 2.8 cm in X during the final
    # two seconds before the command despite low instantaneous velocity.
    samples=[(i*100_000, .028*i/20, 0., .001*i/20, (5,2,1))
             for i in range(21)]
    stable,details=position_stability_window(samples)
    assert not stable
    assert details['axis_excursion_m'][0] == pytest.approx(.028)
    assert 'excursion' in details['reason']


@pytest.mark.parametrize('mutate,reason',[
    (lambda s: s.__setitem__(15,(s[15][0],*s[15][1:4],(5,3,1))), 'reset'),
    (lambda s: s.__setitem__(15,(s[14][0],*s[15][1:]),), 'duplicate or backwards'),
])
def test_position_stability_window_rejects_reset_bad_timestamps_and_gaps(mutate,reason):
    samples=[(i*100_000, 0., 0., 0., (1,2,0)) for i in range(33)]
    mutate(samples)
    stable,details=position_stability_window(samples)
    assert not stable
    assert reason in details['reason']


def test_position_stability_window_rejects_large_measurement_gap():
    times=[i*100_000 for i in range(15)]
    times += [1_800_000+i*100_000 for i in range(16)]
    samples=[(t,0.,0.,0.,(1,2,0)) for t in times]
    stable,details=position_stability_window(samples,min_samples=15)
    assert not stable
    assert 'gap exceeds' in details['reason']


@pytest.mark.parametrize('index,field,value',[
    (2,'arming_state',2),(3,'cs_ev_hgt',False),(3,'cs_ev_pos',False),
    (0,'xy_global',False),(0,'timestamp_sample',1_000_001),
    (1,'timestamp_sample',400_000),(1,'epv',float('nan')),
    (1,'fix_type',2),(1,'vel_m_s',.3),(0,'vx',.3),(0,'z',.7)])
def test_origin_rejects_armed_unfused_stale_noisy_or_moving_inputs(index,field,value):
    values=inputs();setattr(values[index],field,value)
    with pytest.raises(ValueError):origin_candidate(*values)


def initialization_fixture(monkeypatch, post_error=None, always_pending=False):
    """Drive the real two-phase state machine; no ROS process or commands."""
    wall = [0.]; sent = []; last_pending = []; checks = [0]
    local,gps,status,flags,_ = inputs()
    local.z_global=False;local.ref_alt=0.
    local.xy_reset_counter=local.z_reset_counter=local.heading_reset_counter=0
    global_position=NS(alt_valid=False,lat_lon_valid=True,alt=488.)
    fake=NS(data={'local':local,'global':global_position},acks=[],receipts={},
        input_snapshot=lambda:dict(sim_us=int(wall[0]*1e6)),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(wall[0]*1e9))))
    target=[local.ref_lat,local.ref_lon,gps.altitude_msl_m+local.z]
    def candidate():
        if sent:
            checks[0]+=1
            if checks[0]==3 or always_pending:
                last_pending.append(wall[0])
                raise post_error or origin_module.TelemetryPending('stale or future measured position')
        return target
    def send(origin):
        sent.append(origin)
        local.z_global=True;local.ref_alt=origin[2];global_position.alt_valid=True
        fake.acks.append({'result':0})
    fake.candidate=candidate;fake.send=send
    fake.local_position_stable=lambda:(True,dict(reason='',samples=21,duration_us=2_000_000))
    monkeypatch.setattr(origin_module.time,'monotonic',lambda:wall[0])
    def spin(*args,**kwargs):
        wall[0]+=.25
        fake.receipts['global']=wall[0]
    monkeypatch.setattr(origin_module.rclpy,'spin_once',spin)
    return fake, sent, last_pending, wall


def test_one_stale_or_future_callback_restarts_confirmation_without_resending(monkeypatch):
    fake,sent,pending,wall=initialization_fixture(monkeypatch)
    report={'command_sent':False}
    origin_module.Initializer.run(fake,report)
    assert report['confirmed'] and len(sent)==1
    assert report['post_command_pending_count']==1
    assert report['post_command_pending_samples'][0]['sim_us']==pending[0]*1e6
    assert wall[0]-pending[-1]>=1.25  # fresh sample, then full healthy second


def test_persistent_missing_data_times_out_without_resending_or_confirming(monkeypatch):
    fake,sent,_,wall=initialization_fixture(monkeypatch,always_pending=True)
    report={'command_sent':False}
    with pytest.raises(RuntimeError,match='verification timed out'):
        origin_module.Initializer.run(fake,report)
    assert len(sent)==1 and not report.get('confirmed') and wall[0]<=60.25
    assert len(report['post_command_pending_samples'])==16  # bounded report


@pytest.mark.parametrize('error', ['vehicle is not disarmed', 'EV XY/Z fusion has not started',
    'non-finite origin input', 'stationary ground state required'])
def test_real_health_faults_after_command_still_abort_immediately(monkeypatch,error):
    fake,sent,_,wall=initialization_fixture(monkeypatch,post_error=ValueError(error))
    with pytest.raises(ValueError,match=error):
        origin_module.Initializer.run(fake,{'command_sent':False})
    assert len(sent)==1 and wall[0]<5.
