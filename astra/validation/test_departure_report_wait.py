"""Periodic RTL reports must be fresh at handoff without losing hold output."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import ego_sitl_flight as flight


@pytest.mark.parametrize('outcome', ['fresh', 'invalid', 'missing', 'health_failure'])
def test_departure_wait_preserves_health_and_control_until_fresh_report(monkeypatch,outcome):
    now=[0.];sent=[];events=[];checks=[]
    monkeypatch.setattr(flight.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(flight.rclpy,'ok',lambda:True)
    monkeypatch.setattr(flight,'receive_batch',lambda _:now.__setitem__(0,now[0]+.05))
    def health():
        checks.append(now[0])
        if outcome=='health_failure' and now[0]>=.3:
            raise RuntimeError('FAST-LIVO2 receipt age exceeds 250 ms')
    def fault():
        if outcome=='fresh' and now[0]>=.55:return ''
        if outcome=='invalid' and now[0]>=.3:return 'PX4 invalid for RTL: global_position_invalid'
        return 'PX4 failsafe flags stale'
    n=NS(check_safe=health,native_rtl_fault=fault,
         send=lambda target:sent.append((now[0],target)),
         event=lambda name,**kw:events.append((name,kw)))
    if outcome=='fresh':
        flight.await_native_rtl_departure(n,object(),(1.,2.,-1.5,.2))
        assert now[0]>=.55 and now[0]<.7
    else:
        match={'invalid':'global_position_invalid','missing':'flags stale',
               'health_failure':'receipt age exceeds 250 ms'}[outcome]
        with pytest.raises(RuntimeError,match=match):
            flight.await_native_rtl_departure(n,object(),(1.,2.,-1.5,.2))
    assert sent and all(target==(1.,2.,-1.5,.2) for _,target in sent)
    assert len(checks)>len(sent)
    assert now[0]<=1.55
    assert events[0][1]['flags_receipt_limit_s']==.5
