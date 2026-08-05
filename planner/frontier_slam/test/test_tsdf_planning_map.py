"""Tests for the /projected_map path split off the CUBE_LIST walk.

Guards three things measured on 2026-07-31, when the voxel walk had grown to
8.15 s on a 110k-voxel map:

  * the planning map has its own duty gate, so a slow decorative redraw can no
    longer hold the map the planner routes on at a ~30 s period;
  * the surface cloud has its own callback group, so the walk cannot starve the
    topic the wall controller steers on past its 5 s staleness limit;
  * the banded read is vectorised, and agrees voxel-for-voxel with the Python
    walk it replaces.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
import pytest

from frontier_slam.tsdf_mapper import (
    TSDFMapper, _band_index_range, _extract_band_arrays, _extract_voxel_arrays,
)

vdbfusion = pytest.importorskip('vdbfusion')
pytest.importorskip('pyopenvdb')

VOXEL, TRUNC = 0.25, 0.75


def _volume(seed=0):
    """A volume with structure spread over enough Z to make banding matter."""
    rng = np.random.default_rng(seed)
    v = vdbfusion.VDBVolume(voxel_size=VOXEL, sdf_trunc=TRUNC, space_carving=True)
    for origin in ([0.0, 0.0, 0.0], [3.0, 1.0, 2.0]):
        pts = rng.random((800, 3)) * np.array([6.0, 6.0, 5.0]) + np.array([4.0, 4.0, 1.0])
        v.integrate(pts, np.asarray(origin))
    return v


# ── band index arithmetic ────────────────────────────────────────────────────

def test_band_indices_match_the_centre_convention():
    # Centre of index k is k*VOXEL + VOXEL/2, so [0.125, 0.375] is exactly k=0..1.
    assert _band_index_range(0.125, 0.375, VOXEL) == (0, 1)


def test_band_excludes_centres_outside_the_span():
    # 0.2 .. 0.3 contains no centre (0.125 and 0.375 both sit outside) -> empty.
    lo, hi = _band_index_range(0.2, 0.3, VOXEL)
    assert hi < lo


def test_negative_band_indices_round_outward_not_toward_zero():
    lo, hi = _band_index_range(-1.0, -0.5, VOXEL)
    centres = np.arange(lo, hi + 1) * VOXEL + VOXEL / 2.0
    assert np.all(centres >= -1.0) and np.all(centres <= -0.5)


# ── vectorised band read == the Python walk it replaces ──────────────────────

def _walk_band(volume, min_weight, kz_lo, kz_hi):
    """_extract_voxel_arrays, then the band filter, as the old path did it."""
    coords, d, w = _extract_voxel_arrays(volume.tsdf, volume.weights, min_weight)
    if coords is None:
        return None, None, None
    keep = (coords[:, 2] >= kz_lo) & (coords[:, 2] <= kz_hi)
    if not np.any(keep):
        return None, None, None
    return coords[keep], d[keep], w[keep]


def _sorted(coords, d, w):
    order = np.lexsort((coords[:, 2], coords[:, 1], coords[:, 0]))
    return coords[order], d[order], w[order]


@pytest.mark.parametrize('band', [(3, 9), (0, 40), (5, 5)])
def test_band_read_agrees_with_the_python_walk(band):
    v = _volume()
    kz_lo, kz_hi = band
    got = _extract_band_arrays(v.tsdf, v.weights, 2.0, kz_lo, kz_hi)
    want = _walk_band(v, 2.0, kz_lo, kz_hi)

    assert (got[0] is None) == (want[0] is None)
    if got[0] is None:
        return
    gc, gd, gw = _sorted(*got)
    wc, wd, ww = _sorted(*want)
    np.testing.assert_array_equal(gc, wc)
    np.testing.assert_allclose(gd, wd, rtol=0, atol=0)
    np.testing.assert_allclose(gw, ww, rtol=0, atol=0)


def test_band_read_selects_exactly_the_active_set():
    # The weights background is 0.0, so `w >= min_weight` for any min_weight > 0
    # is what makes the dense read equivalent to stepping active voxels.
    v = _volume()
    coords, _, _ = _extract_band_arrays(v.tsdf, v.weights, 1e-6, -10_000, 10_000)
    assert len(coords) == v.tsdf.activeVoxelCount()


def test_band_read_honours_the_weight_floor():
    v = _volume()
    _, _, w = _extract_band_arrays(v.tsdf, v.weights, 2.0, -10_000, 10_000)
    assert np.all(w >= 2.0)


def test_empty_band_returns_the_none_triple():
    v = _volume()
    assert _extract_band_arrays(v.tsdf, v.weights, 2.0, 9_000, 9_100) == (None, None, None)


def test_inverted_band_returns_the_none_triple():
    v = _volume()
    assert _extract_band_arrays(v.tsdf, v.weights, 2.0, 10, 9) == (None, None, None)


def test_band_read_is_bounded_by_the_band_not_the_volume():
    """The point of the split: cost tracks the slab, not the carved volume."""
    v = _volume()
    thin = _extract_band_arrays(v.tsdf, v.weights, 2.0, 6, 8)
    full = _extract_band_arrays(v.tsdf, v.weights, 2.0, -10_000, 10_000)
    assert thin[0] is not None
    assert len(thin[0]) < len(full[0])


# ── the CUBE_LIST gate no longer holds the planning map ──────────────────────

class _Recorder:
    """A mapper stub exercising only the two publish entry points' gating."""

    def __init__(self):
        self._pyopenvdb = True
        self._map_revision = 7
        self._voxels_revision = 0
        self._planning_revision = 0
        self._voxels_cache = None
        self._planning_cache = None
        self._monotonic = lambda: 100.0
        self._viz_cap = ''
        self._viz_cap_last = ''
        self.rebuilt = []

    _voxels_gate = property(lambda self: self._vg)
    _planning_gate = property(lambda self: self._pg)


def _stub(voxels_ready, planning_ready):
    m = _Recorder()
    m._vg = type('G', (), {'ready': lambda _s, _n: voxels_ready})()
    m._pg = type('G', (), {'ready': lambda _s, _n: planning_ready})()
    m._rebuild_voxel_cache = lambda started: m.rebuilt.append('voxels')
    m._rebuild_planning_cache = lambda started: m.rebuilt.append('planning')
    m._publish_viz_cap = lambda text: TSDFMapper._publish_viz_cap(m, text)
    m._viz_cap_pub = type(
        'P', (), {'publish': lambda _s, x: m.cap_published.append(x.data)})()
    m.cap_published = []
    return m


def test_planning_map_rebuilds_while_the_cube_list_gate_is_blocked():
    """The regression this split exists for: an 8 s grid walk shut the gate for
    ~24 s and took /projected_map down with it."""
    m = _stub(voxels_ready=False, planning_ready=True)
    TSDFMapper._publish_voxels(m)
    TSDFMapper._publish_projected_map(m)
    assert m.rebuilt == ['planning']


def test_cube_list_still_rebuilds_when_its_own_gate_opens():
    m = _stub(voxels_ready=True, planning_ready=False)
    TSDFMapper._publish_voxels(m)
    TSDFMapper._publish_projected_map(m)
    assert m.rebuilt == ['voxels']


def test_neither_rebuilds_once_its_revision_is_current():
    m = _stub(voxels_ready=True, planning_ready=True)
    m._voxels_revision = m._planning_revision = m._map_revision
    TSDFMapper._publish_voxels(m)
    TSDFMapper._publish_projected_map(m)
    assert m.rebuilt == []


def test_publish_voxels_unpacks_a_triple():
    """A stale 4-tuple anywhere would raise here rather than at runtime."""
    m = _stub(voxels_ready=False, planning_ready=False)
    published = []

    def _msg():
        return type('M', (), {'header': type('H', (), {'stamp': None})()})()

    m._voxels_cache = (_msg(), _msg(), _msg())
    m._solid_cloud_pub = type('P', (), {'publish': lambda _s, x: published.append(x)})()
    m._voxels_pub = m._solid_cloud_pub
    m._free_cloud_pub = m._solid_cloud_pub
    m.get_clock = lambda: type('C', (), {'now': lambda _s: type(
        'T', (), {'to_msg': lambda _s2: 'stamp'})()})()
    TSDFMapper._publish_voxels(m)
    assert len(published) == 3


def test_cap_warning_goes_to_its_own_topic_and_only_on_change():
    """The HUD row is state: republishing it every tick would flood /tsdf/viz_cap."""
    m = _stub(voxels_ready=False, planning_ready=False)

    def _msg():
        return type('M', (), {'header': type('H', (), {'stamp': None})()})()

    m._voxels_cache = (_msg(), _msg(), None)
    m._solid_cloud_pub = m._voxels_pub = type(
        'P', (), {'publish': lambda _s, _x: None})()
    m.get_clock = lambda: type('C', (), {'now': lambda _s: type(
        'T', (), {'to_msg': lambda _s2: 'stamp'})()})()

    m._viz_cap = 'VOXEL VIEW CAPPED - showing 1 of 2 (50%)'
    TSDFMapper._publish_voxels(m)
    TSDFMapper._publish_voxels(m)
    assert m.cap_published == [m._viz_cap]

    m._viz_cap = ''
    TSDFMapper._publish_voxels(m)
    assert m.cap_published[-1] == ''


# ── the gate charges work, not lock contention ───────────────────────────────

class _SlowLock:
    """A lock that takes `wait` seconds to acquire and none to hold."""

    def __init__(self, clock, wait):
        self._clock, self._wait = clock, wait

    def __enter__(self):
        self._clock.t += self._wait

    def __exit__(self, *exc):
        return False


class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _CountingGate:
    last_s = None

    def ready(self, _now):
        return True

    def record(self, _now, elapsed):
        self.last_s = elapsed


class _PlanningStub:
    """Just enough of TSDFMapper for _rebuild_planning_cache to run."""

    def __init__(self, lock_wait):
        self.clock = _Clock()
        self._monotonic = self.clock
        self._volume_lock = _SlowLock(self.clock, lock_wait)
        self._planning_gate = _CountingGate()
        self._map_revision = 3
        self._planning_revision = 0
        self._planning_cache = None
        self._projected_map_band = 1.0
        self._voxel_size = 0.25
        self._min_weight = 2.0
        self._volumes = [type('V', (), {'tsdf': None, 'weights': None})()]
        self._band_center_z = lambda: 8.0
        self._build_projected_map_msg = lambda *a: None
        self.get_logger = lambda: type('L', (), {'info': lambda *a, **k: None})()
        self.get_clock = lambda: type('C', (), {'now': lambda _s: type(
            'T', (), {'to_msg': lambda _s2: None})()})()


def _charge(monkeypatch, lock_wait, work):
    import frontier_slam.tsdf_mapper as tm
    stub = _PlanningStub(lock_wait)

    def _extract(*_a, **_k):
        stub.clock.t += work
        return None, None, None

    monkeypatch.setattr(tm, '_extract_band_arrays', _extract)
    tm.TSDFMapper._rebuild_planning_cache(stub, started=0.0)
    return stub._planning_gate.last_s


def test_gate_is_charged_the_read_not_the_lock_wait(monkeypatch):
    """A 4 s block behind the CUBE_LIST walk must not mute the planning map;
    only the read itself counts against the duty budget."""
    assert _charge(monkeypatch, lock_wait=4.0, work=0.1) == pytest.approx(0.1)


def test_a_genuinely_slow_read_is_still_charged_in_full(monkeypatch):
    assert _charge(monkeypatch, lock_wait=0.0, work=2.5) == pytest.approx(2.5)


# ── callback groups on a real node ───────────────────────────────────────────

def test_the_three_viz_timers_are_in_three_callback_groups():
    """Sharing one MutuallyExclusiveCallbackGroup meant a grid walk blocked
    /tsdf/surface_cloud outright — past the wall controller's 5 s staleness
    limit, which dropped its wall-side lock for 13-28% of a run."""
    rclpy = pytest.importorskip('rclpy')
    rclpy.init(args=['--ros-args', '-p', 'publish_projected_map:=true'])
    try:
        node = TSDFMapper()
        try:
            groups = {t.callback.__name__: id(t.callback_group)
                      for t in node.timers}
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

    for name in ('_publish_surface', '_publish_voxels', '_publish_projected_map'):
        assert name in groups, f'{name} has no timer'
    assert len(set(groups[n] for n in (
        '_publish_surface', '_publish_voxels', '_publish_projected_map'))) == 3
