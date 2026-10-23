"""Optional adapter-local tracing; no scheduling or sensor-data changes."""
import gc
import json
from pathlib import Path
import resource
import sys
import threading
import time
import traceback


def scheduler_wait_ns():
    try:
        return int(Path('/proc/thread-self/schedstat').read_text().split()[1])
    except (OSError, ValueError, IndexError):
        return None


def info_value(info, key):
    return int((info.get(key, 0) if isinstance(info, dict) else getattr(info, key, 0)) or 0)


def stale_lidar_expired(received_wall, now_wall, max_age_sec=0.3):
    """Return true when a buffered scan is too old to enter the estimator."""
    return now_wall - received_wall > max_age_sec


def discard_expired_lidar_head(queue, now_wall, max_age_sec=0.3):
    """Drop one stale (stamp, receipt_wall, payload) scan; leave newer scans queued."""
    if not queue or not stale_lidar_expired(queue[0][1], now_wall, max_age_sec):
        return False
    queue.popleft()
    return True


class InputCallbackAudit:
    def __init__(self, record, stall_threshold_sec=0.05):
        self.record = record
        self.gc_starts = {}
        self.gc_callback = self.on_gc
        gc.callbacks.append(self.gc_callback)
        self.stall_threshold_ns = int(stall_threshold_sec * 1e9)
        self.active_lock = threading.Lock()
        self.active = {}
        self.reported = set()
        self.stop_watchdog = threading.Event()
        self.watchdog = threading.Thread(target=self.watch_callbacks,
                                         name='sensor-callback-watchdog', daemon=True)
        self.watchdog.start()

    def thread_stacks(self):
        frames = sys._current_frames()
        result = []
        for thread in threading.enumerate():
            ident = thread.native_id
            frame = frames.get(thread.ident)
            if ident is None or frame is None:
                continue
            result.append(dict(thread_id=ident, name=thread.name,
                               stack=''.join(traceback.format_stack(frame)[-12:])))
        return result

    def watch_callbacks(self):
        while not self.stop_watchdog.wait(0.01):
            now = time.monotonic_ns()
            with self.active_lock:
                stalled = [(thread_id, dict(item)) for thread_id, item in self.active.items()
                           if now-item['wall_start_ns'] >= self.stall_threshold_ns
                           and thread_id not in self.reported]
                self.reported.update(thread_id for thread_id, _ in stalled)
            if stalled:
                stacks = self.thread_stacks()
                with self.active_lock:
                    active_callbacks = [dict(thread_id=tid, **active)
                        for tid, active in self.active.items()]
                for thread_id, item in stalled:
                    self.record('callback_stall_snapshot', detail=json.dumps(dict(
                        label=item['label'], thread_id=thread_id,
                        wall_start_ns=item['wall_start_ns'], snapshot_wall_ns=now,
                        active_callbacks=active_callbacks, threads=stacks)))

    def on_gc(self, phase, info):
        generation = info['generation']
        if phase == 'start':
            self.gc_starts[generation] = time.monotonic_ns()
        elif phase == 'stop':
            start = self.gc_starts.pop(generation, None)
            if start is not None:
                self.record('adapter_gc', detail=json.dumps(dict(
                    generation=generation, wall_start_ns=start, wall_end_ns=time.monotonic_ns(),
                    collected=info['collected'], uncollectable=info['uncollectable'])))

    def call(self, callback, label, args, info=None):
        wall = time.monotonic_ns()
        system = time.time_ns()
        thread_id = threading.get_native_id()
        with self.active_lock:
            self.active[thread_id] = dict(label=label, wall_start_ns=wall)
        cpu = time.thread_time_ns()
        wait = scheduler_wait_ns()
        faults = resource.getrusage(resource.RUSAGE_THREAD).ru_majflt
        received = info_value(info, 'received_timestamp')
        self.record('callback_dispatch', detail=json.dumps(dict(
            label=label, wall_start_ns=wall, callback_system_ns=system,
            thread_id=thread_id,
            dds_source_system_ns=info_value(info, 'source_timestamp'),
            dds_received_system_ns=received)))
        if received and system-received >= self.stall_threshold_ns:
            self.record('callback_queue_stall_snapshot', detail=json.dumps(dict(
                label=label, thread_id=thread_id, dds_wait_ms=(system-received)/1e6,
                callback_system_ns=system, dds_received_system_ns=received,
                threads=self.thread_stacks())))
        try:
            return callback(*args)
        finally:
            with self.active_lock:
                self.active.pop(thread_id, None)
                self.reported.discard(thread_id)
            finish_wait = scheduler_wait_ns()
            self.record('callback_span', detail=json.dumps(dict(
                label=label, wall_start_ns=wall, wall_end_ns=time.monotonic_ns(),
                thread_cpu_ns=time.thread_time_ns()-cpu,
                scheduler_wait_ns=None if wait is None or finish_wait is None else finish_wait-wait,
                major_faults=resource.getrusage(resource.RUSAGE_THREAD).ru_majflt-faults)))

    def subscription(self, callback, label):
        def traced(msg, info):
            return self.call(callback, label, (msg,), info)
        return traced

    def timer(self, callback, label):
        def traced():
            return self.call(callback, label, ())
        return traced

    def close(self):
        self.stop_watchdog.set()
        self.watchdog.join(timeout=1.0)
        if self.gc_callback in gc.callbacks:
            gc.callbacks.remove(self.gc_callback)
