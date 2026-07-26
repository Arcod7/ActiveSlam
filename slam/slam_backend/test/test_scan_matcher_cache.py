"""The prepared-cloud cache must be a pure speedup, not a behaviour change.

Loop-closure detection scores one source against several candidate targets. It
used to rebuild the target KD-tree and re-downsample the source on every call;
prepare() hoists both out. These tests pin that the cached path returns the same
registration as the uncached one.
"""
import numpy as np
import pytest

from slam_backend.scan_matcher import ScanMatcher


def ridged_cloud(n=800, shift=(0.0, 0.0, 0.0), seed=0):
    """A cloud with enough structure for GICP to lock onto in all three axes."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-2.0, 2.0, size=(n, 3))
    pts[:, 2] = np.sin(pts[:, 0] * 1.5) + 0.3 * np.cos(pts[:, 1] * 2.0)
    return pts + np.asarray(shift)


@pytest.fixture
def matcher():
    return ScanMatcher(max_correspondence_dist=1.0, downsampling_resolution=0.1)


def test_prepared_align_matches_unprepared(matcher):
    target = ridged_cloud(seed=1)
    source = ridged_cloud(shift=(0.15, -0.1, 0.0), seed=1)

    plain = matcher.align(source, target)
    cached = matcher.align(source, target,
                           source_prepared=matcher.prepare(source),
                           target_prepared=matcher.prepare(target))

    assert np.allclose(plain['T_target_source'], cached['T_target_source'], atol=1e-9)
    assert plain['num_inliers'] == cached['num_inliers']
    assert plain['num_source_points'] == cached['num_source_points']
    assert cached['error'] == pytest.approx(plain['error'], rel=1e-9)


def test_one_prepared_target_serves_repeated_aligns(matcher):
    """The cached target is reused across candidates; it must not be consumed."""
    target_prepared = matcher.prepare(ridged_cloud(seed=2))
    source = ridged_cloud(shift=(0.1, 0.0, 0.0), seed=2)

    first = matcher.align(source, None, target_prepared=target_prepared)
    second = matcher.align(source, None, target_prepared=target_prepared)

    assert np.allclose(first['T_target_source'], second['T_target_source'], atol=1e-12)


def test_preparing_only_one_side_is_allowed(matcher):
    """The sequential match caches the source only; the target may be raw."""
    target = ridged_cloud(seed=3)
    source = ridged_cloud(shift=(0.1, 0.05, 0.0), seed=3)

    plain = matcher.align(source, target)
    half = matcher.align(source, target, source_prepared=matcher.prepare(source))

    assert np.allclose(plain['T_target_source'], half['T_target_source'], atol=1e-9)


def test_initial_guess_still_applies_through_the_cache(matcher):
    """Registration must start from the supplied guess, not identity."""
    target = ridged_cloud(seed=4)
    source = ridged_cloud(shift=(0.4, 0.0, 0.0), seed=4)
    guess = np.eye(4)
    guess[0, 3] = -0.4

    result = matcher.align(source, target, guess,
                           source_prepared=matcher.prepare(source),
                           target_prepared=matcher.prepare(target))

    assert result['converged']
    assert result['T_target_source'][0, 3] == pytest.approx(-0.4, abs=0.15)
