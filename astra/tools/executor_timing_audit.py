"""Bounded diagnostic tracing; timestamps are evidence, never flight inputs."""
from collections import defaultdict, deque
import gc
import json
from pathlib import Path
import struct
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


class _AuditSpan:
    """Reuse one class: defining a type per check creates cyclic GC garbage."""
    __slots__ = ('audit','label','wall','cpu','wait')

    def __init__(self,audit,label):
        self.audit,self.label=audit,label

    def __enter__(self):
        self.wall=time.monotonic_ns();self.cpu=time.thread_time_ns()
        self.wait=scheduler_wait_ns()
        return self

    def __exit__(self,*_):
        finish_wait=scheduler_wait_ns()
        self.audit.record(dict(label=self.label,wall_start_ns=self.wall,wall_end_ns=time.monotonic_ns(),
            wall_duration_ns=time.monotonic_ns()-self.wall,cpu_duration_ns=time.thread_time_ns()-self.cpu,
            scheduler_wait_ns=None if self.wait is None or finish_wait is None else finish_wait-self.wait))


class CompactOdomHistory:
    """Preallocate the existing diagnostic window instead of retaining dicts.

    Only Odom callbacks have this fixed schema. Decode dictionaries during
    offline export, preserving nanoseconds, optional measurement time, labels
    and the same chronological 65536-row retention policy.
    """
    fields = ('wall_start_ns', 'measurement_ns', 'ros_clock_before_ns',
              'dds_received_system_ns', 'dds_source_system_ns',
              'callback_start_system_ns', 'wall_end_ns', 'wall_duration_ns',
              'cpu_duration_ns', 'ros_clock_after_ns')
    layout = struct.Struct('<I10q?')
    missing = -(1 << 63)

    def __init__(self, capacity=65536):
        if not isinstance(capacity, int) or isinstance(capacity, bool) or capacity <= 0:
            raise ValueError('Odom history capacity must be a positive integer')
        self.maxlen = capacity
        self.storage = bytearray(capacity * self.layout.size)
        self.labels = []
        self.label_ids = {}
        self.count = 0

    def append(self, row):
        label = row['label']
        if label not in self.label_ids:
            self.label_ids[label] = len(self.labels)
            self.labels.append(label)
        values = [self.missing if row[key] is None else row[key] for key in self.fields]
        self.layout.pack_into(self.storage, (self.count % self.maxlen) * self.layout.size,
                              self.label_ids[label], *values, row['dds_metadata_available'])
        self.count += 1

    def __len__(self):
        return min(self.count, self.maxlen)

    def __iter__(self):
        for index in range(max(0, self.count - self.maxlen), self.count):
            values = self.layout.unpack_from(self.storage, (index % self.maxlen) * self.layout.size)
            row = dict(label=self.labels[values[0]], topic='/fast_livo2/odom')
            row.update((key, None if value == self.missing else value)
                       for key, value in zip(self.fields, values[1:-1]))
            row['dds_metadata_available'] = values[-1]
            yield row


class ExecutorTimingAudit:
    def __init__(self):
        self.tail=deque(maxlen=8192)
        self.batch_tail=deque(maxlen=2048)
        # RViz ground editing has no timeout. Keep diagnostic memory bounded
        # even if an operator leaves the session waiting for hours.
        self.odom_callbacks=CompactOdomHistory()
        self.slow=deque(maxlen=1024)
        self.stats=defaultdict(lambda:dict(count=0,total_wall_ns=0,total_cpu_ns=0,max_wall_ns=0))
        self.installed=[]
        self.gc_events=deque(maxlen=256)
        self.gc_stats=defaultdict(lambda:dict(count=0,total_wall_ns=0,max_wall_ns=0))
        self.gc_starts={}
        self._gc_callback=self._record_gc

    def _record_gc(self,phase,info):
        generation=info['generation']
        if phase=='start':
            self.gc_starts[generation]=(time.monotonic_ns(),time.thread_time_ns())
        elif phase=='stop':
            start=self.gc_starts.pop(generation,None)
            if start is None:return
            cpu_end=time.thread_time_ns()
            end=time.monotonic_ns()
            row=dict(generation=generation,wall_start_ns=start[0],wall_end_ns=end,
                     wall_duration_ns=end-start[0],cpu_duration_ns=cpu_end-start[1],
                     collected=info['collected'],uncollectable=info['uncollectable'])
            self.gc_events.append(row)
            stats=self.gc_stats[generation]
            stats['count']+=1;stats['total_wall_ns']+=row['wall_duration_ns']
            stats['max_wall_ns']=max(stats['max_wall_ns'],row['wall_duration_ns'])

    def record(self,row):
        self.tail.append(row)
        s=self.stats[row['label']];s['count']+=1
        s['total_wall_ns']+=row['wall_duration_ns'];s['total_cpu_ns']+=row['cpu_duration_ns']
        s['max_wall_ns']=max(s['max_wall_ns'],row['wall_duration_ns'])
        if row['wall_duration_ns']>=5_000_000:self.slow.append(row)
        if row.get('topic')=='/fast_livo2/odom':self.odom_callbacks.append(row)

    def install(self,node,with_metadata=True):
        if self._gc_callback not in gc.callbacks:gc.callbacks.append(self._gc_callback)
        installed={id(entity) for entity,_ in self.installed}
        for sub in node.subscriptions:
            if id(sub) in installed:continue
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
            if id(timer) in installed:continue
            original=timer.callback
            if original is None:continue
            label=node.get_name()+':timer:'+getattr(original,'__name__','unknown')
            def make_timer(callback,label):
                def traced():
                    with self.span(label):return callback()
                return traced
            self.installed.append((timer,original));timer.callback=make_timer(original,label)

    def span(self,label):
        return _AuditSpan(self,label)

    def restore(self):
        if self._gc_callback in gc.callbacks:gc.callbacks.remove(self._gc_callback)
        for entity,callback in self.installed:entity.callback=callback
        self.installed.clear()

    def save(self,path):
        Path(path).write_text(json.dumps(dict(scope='Diagnostic only; flight gates and automatic GC settings retained',
            odom_history=dict(storage='preallocated packed nanoseconds', capacity=self.odom_callbacks.maxlen,
                total_count=self.odom_callbacks.count, retained_count=len(self.odom_callbacks),
                overwritten_count=max(0,self.odom_callbacks.count-self.odom_callbacks.maxlen),
                allocated_bytes=len(self.odom_callbacks.storage)),
            callback_statistics=dict(self.stats),tail=list(self.tail),batch_tail=list(self.batch_tail),
            slow_callbacks=list(self.slow),odom_callbacks=list(self.odom_callbacks),
            gc_statistics=dict(self.gc_stats),gc_tail=list(self.gc_events)),indent=2)+'\n')
