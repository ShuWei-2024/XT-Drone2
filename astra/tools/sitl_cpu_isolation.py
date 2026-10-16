"""Per-run CPU affinity; never changes global scheduling or PX4 parameters."""
import os
from pathlib import Path


def split_physical_cores(topology):
    """Keep SMT siblings together, reserve two full cores for sensor estimation."""
    groups={}
    for cpu,identity in topology.items():groups.setdefault(identity,[]).append(cpu)
    if len(groups)<4:
        return dict(enabled=False,reason='at least four available physical cores required')
    identities=sorted(groups)
    estimator=sorted(cpu for key in identities[-2:] for cpu in groups[key])
    simulation=sorted(cpu for key in identities[:-2] for cpu in groups[key])
    return dict(enabled=True,estimator_cpus=estimator,simulation_cpus=simulation,
                scope='owned SITL child processes only; inherited by child threads')


def cpu_plan():
    topology={}
    for cpu in os.sched_getaffinity(0):
        root=Path(f'/sys/devices/system/cpu/cpu{cpu}/topology')
        topology[cpu]=(int((root/'physical_package_id').read_text()),
                       int((root/'core_id').read_text()))
    return split_physical_cores(topology)


def command_for_role(command,role,plan):
    if not plan['enabled']:return list(command)
    cpus=plan['estimator_cpus'] if role=='livo' else plan['simulation_cpus']
    return ['taskset','--cpu-list',','.join(map(str,cpus)),*command]
