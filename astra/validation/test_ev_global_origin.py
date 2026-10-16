"""Guard GNSS origin initialization independently from the EV local height datum."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from initialize_ev_global_origin import origin_candidate, absolute_origin_valid
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
