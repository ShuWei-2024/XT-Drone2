from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from sitl_cpu_isolation import split_physical_cores,command_for_role


def test_smt_siblings_and_sparse_available_cpus_stay_together():
    topology={0:(0,0),4:(0,0),1:(0,1),5:(0,1),2:(0,2),6:(0,2),3:(0,3),7:(0,3)}
    plan=split_physical_cores(topology)
    assert plan['estimator_cpus']==[2,3,6,7]
    assert plan['simulation_cpus']==[0,1,4,5]
    assert command_for_role(['ros2','launch'],'livo',plan)==['taskset','--cpu-list','2,3,6,7','ros2','launch']
    assert command_for_role(['gz','sim'],'simulator',plan)==['taskset','--cpu-list','0,1,4,5','gz','sim']
    del topology[7]
    assert split_physical_cores(topology)['estimator_cpus']==[2,3,6]


def test_insufficient_physical_cores_does_not_pin_any_process():
    plan=split_physical_cores({0:(0,0),1:(0,0),2:(0,1),3:(0,1)})
    assert not plan['enabled']
    assert command_for_role(['ros2','launch'],'livo',plan)==['ros2','launch']
