"""Focused tests for TSDF mapper's bounded exact-time TF deferral queue."""
from collections import deque
from types import SimpleNamespace

from frontier_slam.tsdf_mapper import TSDFMapper


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message, **_kwargs):
        self.warnings.append(message)


class _Future:
    def __init__(self, result=None, done=False, exception=None):
        self._result = result
        self._done = done
        self._exception = exception
        self._cancelled = False

    def done(self):
        return self._done

    def cancelled(self):
        return self._cancelled

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def set_result(self, result):
        self._result = result
        self._done = True

    def cancel(self):
        self._cancelled = True
        self._done = True


def _msg(stamp):
    header = SimpleNamespace(frame_id='camera', stamp=stamp)
    return SimpleNamespace(header=header)


def _mapper(queue_size=3, timeout=0.5):
    mapper = TSDFMapper.__new__(TSDFMapper)
    mapper._tf_queue = deque()
    mapper._tf_queue_size = queue_size
    mapper._tf_wait_timeout_s = timeout
    mapper._now = 0.0
    mapper._monotonic = lambda: mapper._now
    mapper._cloud_received = 0
    mapper._cloud_integrated = 0
    mapper._tf_deferred = 0
    mapper._tf_recovered = 0
    mapper._tf_expired = 0
    mapper._tf_failed = 0
    mapper._tf_overflow = 0
    mapper._logger = _Logger()
    mapper.get_logger = lambda: mapper._logger
    mapper.integrated = []
    mapper._integrate_cloud = lambda msg, tf: mapper.integrated.append(
        (msg.header.stamp, tf)) or True
    return mapper


def test_immediate_transform_integrates_without_queueing():
    mapper = _mapper()
    mapper._lookup_cloud_transform = lambda _msg: 'tf-now'

    mapper._cloud_cb(_msg(1))

    assert mapper.integrated == [(1, 'tf-now')]
    assert not mapper._tf_queue
    assert mapper._cloud_received == mapper._cloud_integrated == 1
    assert mapper._tf_deferred == 0


def test_missing_transform_is_recovered_after_callback_returns():
    mapper = _mapper()
    future = _Future()
    mapper._lookup_cloud_transform = lambda _msg: None
    mapper._wait_for_cloud_transform = lambda _msg: future

    mapper._cloud_cb(_msg(2))
    assert len(mapper._tf_queue) == 1
    assert mapper.integrated == []

    mapper._now = 0.1
    future.set_result('tf-later')
    mapper._tf_queue_tick()

    assert mapper.integrated == [(2, 'tf-later')]
    assert not mapper._tf_queue
    assert mapper._tf_deferred == mapper._tf_recovered == 1
    assert mapper._tf_expired == 0


def test_fifo_waits_for_oldest_transform_before_newer_cloud():
    mapper = _mapper()
    futures = {1: _Future(), 2: _Future(result='tf-newer', done=True)}
    mapper._lookup_cloud_transform = lambda _msg: None
    mapper._wait_for_cloud_transform = lambda msg: futures[msg.header.stamp]

    mapper._cloud_cb(_msg(1))
    mapper._cloud_cb(_msg(2))
    mapper._now = 0.1
    mapper._tf_queue_tick()

    assert mapper.integrated == []
    assert [item[0].header.stamp for item in mapper._tf_queue] == [1, 2]


def test_expired_cloud_is_dropped_then_next_ready_cloud_integrates():
    mapper = _mapper(timeout=0.5)
    futures = {1: _Future(), 2: _Future(result='tf-2', done=True)}
    mapper._lookup_cloud_transform = lambda _msg: None
    mapper._wait_for_cloud_transform = lambda msg: futures[msg.header.stamp]
    mapper._cloud_cb(_msg(1))
    mapper._cloud_cb(_msg(2))

    mapper._now = 0.6
    mapper._tf_queue_tick()

    assert mapper.integrated == [(2, 'tf-2')]
    assert mapper._tf_expired == 1
    assert mapper._tf_recovered == 1
    assert futures[1].cancelled()
    assert mapper._logger.warnings == [
        'Exact-time TF did not arrive before deadline; dropping scan']


def test_queue_overflow_drops_oldest_and_stays_bounded():
    mapper = _mapper(queue_size=2)
    mapper._lookup_cloud_transform = lambda _msg: None
    futures = {stamp: _Future() for stamp in (1, 2, 3)}
    mapper._wait_for_cloud_transform = lambda msg: futures[msg.header.stamp]

    mapper._cloud_cb(_msg(1))
    mapper._cloud_cb(_msg(2))
    mapper._cloud_cb(_msg(3))

    assert [item[0].header.stamp for item in mapper._tf_queue] == [2, 3]
    assert mapper._tf_overflow == 1
    assert futures[1].cancelled()
    assert mapper._logger.warnings == ['Cloud/TF queue full; dropping oldest scan']


def test_failed_async_wait_drops_cloud_without_crashing_queue_tick():
    mapper = _mapper()
    future = _Future(done=True, exception=RuntimeError('TF buffer reset'))
    mapper._lookup_cloud_transform = lambda _msg: None
    mapper._wait_for_cloud_transform = lambda _msg: future

    mapper._cloud_cb(_msg(1))
    mapper._tf_queue_tick()

    assert not mapper._tf_queue
    assert mapper._tf_failed == 1
    assert mapper._cloud_integrated == 0
    assert mapper._logger.warnings == [
        'Exact-time TF wait failed (RuntimeError); dropping scan']
