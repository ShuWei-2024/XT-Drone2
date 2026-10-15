"""Guard GNSS origin initialization independently from the EV local height datum."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from initialize_ev_global_origin import origin_candidate, absolute_origin_valid


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
