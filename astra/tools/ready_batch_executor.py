"""Drain ready work with the standard ROS single-threaded callback semantics."""
import time

from rclpy.executors import SingleThreadedExecutor


class ReadyBatchExecutor(SingleThreadedExecutor):
    def wait_for_ready_callbacks(self, *args, **kwargs):
        work = super().wait_for_ready_callbacks(*args, **kwargs)
        self._batch_had_work = True
        return work

    def spin_once_ready(self, timeout_sec):
        self._batch_had_work = False
        # Keep ROS exception propagation, shutdown handling and spin ownership.
        super().spin_once(timeout_sec=timeout_sec)
        return self._batch_had_work


class BoundedEventsExecutor:
    """Optional Jazzy event executor, with a bounded single-threaded drain."""
    def __init__(self):
        from rclpy.experimental.events_executor import EventsExecutor
        self._executor = EventsExecutor()

    def __getattr__(self, name):
        return getattr(self._executor, name)

    def receive_batch(self, wait_sec, drain_sec, max_extra_calls):
        self._executor.spin_once(timeout_sec=wait_sec)
        deadline = time.monotonic() + drain_sec
        spins = 1
        # The C++ API returns no work indicator. Its empty event-queue check is
        # cheap; cap both count and wall time, with no extra blocking idle waits.
        for _ in range(max_extra_calls):
            if time.monotonic() >= deadline:
                break
            self._executor.spin_once(timeout_sec=0.)
            spins += 1
        return dict(spin_calls=spins, ready_handlers=None, idle_wait=None,
                    budget_exhausted=time.monotonic() >= deadline)


def receive_ready_batch(executor, wait_sec=.002, drain_sec=.004, max_extra_calls=32):
    if isinstance(executor, BoundedEventsExecutor):
        return executor.receive_batch(wait_sec, drain_sec, max_extra_calls)
    # Reuse identical wait arguments so rclpy preserves its ready iterator across
    # calls AND batches. Switching .002 -> 0 discards the previous ready iterator.
    # Stop at the first idle wait instead of rebuilding empty wait sets repeatedly.
    had_work = executor.spin_once_ready(timeout_sec=wait_sec)
    deadline = time.monotonic() + drain_sec
    spins = 1
    callbacks = int(had_work)
    for _ in range(max_extra_calls):
        if not had_work or time.monotonic() >= deadline:
            break
        had_work = executor.spin_once_ready(timeout_sec=wait_sec)
        spins += 1
        callbacks += int(had_work)
    return dict(spin_calls=spins, ready_handlers=callbacks, idle_wait=not had_work,
                budget_exhausted=time.monotonic() >= deadline)
