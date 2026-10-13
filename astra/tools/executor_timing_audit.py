"""Bounded diagnostic tracing; timestamps are evidence, never flight inputs."""
from collections import defaultdict, deque
import json
from pathlib import Path
import time


def scheduler_wait_ns():
    try:return int(Path('/proc/thread-self/schedstat').read_text().split()[1])
    except (OSError,ValueError,IndexError):return None


def measurement_ns(msg):
    stamp=getattr(getattr(msg,'header',None),'stamp',None)
    if stamp is None:stamp=getattr(msg,'clock',None)
    if stamp is not None:return stamp.sec*10**9+stamp.nanosec
    sample=getattr(msg,'timestamp_sample',None)
    return int(sample)*1000 if sample is not None else None


class ExecutorTimingAudit:
    def __init__(self):
        self.tail=deque(maxlen=8192)
        self.batch_tail=deque(maxlen=2048)
        self.odom_callbacks=[]
        self.slow=deque(maxlen=1024)
        self.stats=defaultdict(lambda:dict(count=0,total_wall_ns=0,total_cpu_ns=0,max_wall_ns=0))
        self.installed=[]

    def record(self,row):
        self.tail.append(row)
        s=self.stats[row['label']];s['count']+=1
        s['total_wall_ns']+=row['wall_duration_ns'];s['total_cpu_ns']+=row['cpu_duration_ns']
        s['max_wall_ns']=max(s['max_wall_ns'],row['wall_duration_ns'])
        if row['wall_duration_ns']>=5_000_000:self.slow.append(row)
        if row.get('topic')=='/fast_livo2/odom':self.odom_callbacks.append(row)

    def install(self,node,with_metadata=True):
        for sub in node.subscriptions:
            original=sub.callback;topic=sub.topic_name
            label=node.get_name()+':'+topic
            # Existing subscriptions in these nodes are message-only. Request
            # DDS metadata while preserving their callback argument exactly.
            def make(callback,label,topic):
                def record_call(msg,info=None):
                    wall=time.monotonic_ns();cpu=time.thread_time_ns();system=time.time_ns()
                    received=(info.get('received_timestamp',0) if isinstance(info,dict)
                              else getattr(info,'received_timestamp',0))
                    source=(info.get('source_timestamp',0) if isinstance(info,dict)
                            else getattr(info,'source_timestamp',0))
                    row=dict(label=label,topic=topic,wall_start_ns=wall,
                             measurement_ns=measurement_ns(msg),
                             ros_clock_before_ns=node.get_clock().now().nanoseconds,
                             dds_received_system_ns=int(received),dds_source_system_ns=int(source),
                             dds_metadata_available=info is not None,
                             callback_start_system_ns=system)
                    try:return callback(msg)
                    finally:
                        row.update(wall_end_ns=time.monotonic_ns(),
                            wall_duration_ns=time.monotonic_ns()-wall,
                            cpu_duration_ns=time.thread_time_ns()-cpu,
                            ros_clock_after_ns=node.get_clock().now().nanoseconds)
                        self.record(row)
                if with_metadata:
                    def traced(msg,info):return record_call(msg,info)
                else:
                    # Jazzy EventsExecutor cannot supply MessageInfo; requesting
                    # it raises SystemError. Keep callback timing and mark the
                    # missing transport evidence rather than inventing timestamps.
                    def traced(msg):return record_call(msg)
                return traced
            self.installed.append((sub,original));sub.callback=make(original,label,topic)
        for timer in node.timers:
            original=timer.callback
            if original is None:continue
            label=node.get_name()+':timer:'+getattr(original,'__name__','unknown')
            def make_timer(callback,label):
                def traced():
                    with self.span(label):return callback()
                return traced
            self.installed.append((timer,original));timer.callback=make_timer(original,label)

    def span(self,label):
        audit=self
        class Span:
            def __enter__(self):
                self.wall=time.monotonic_ns();self.cpu=time.thread_time_ns()
            def __exit__(self,*_):
                audit.record(dict(label=label,wall_start_ns=self.wall,wall_end_ns=time.monotonic_ns(),
                    wall_duration_ns=time.monotonic_ns()-self.wall,cpu_duration_ns=time.thread_time_ns()-self.cpu))
        return Span()

    def restore(self):
        for entity,callback in self.installed:entity.callback=callback
        self.installed.clear()

    def save(self,path):
        Path(path).write_text(json.dumps(dict(scope='Diagnostic only; original queue depths and flight gates retained',
            callback_statistics=dict(self.stats),tail=list(self.tail),batch_tail=list(self.batch_tail),
            slow_callbacks=list(self.slow),odom_callbacks=self.odom_callbacks),indent=2)+'\n')
