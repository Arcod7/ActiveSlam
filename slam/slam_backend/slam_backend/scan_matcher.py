import numpy as np
import small_gicp


class ScanMatcher:
    """Point cloud registration wrapper around small_gicp (VGICP-capable GICP).

    small_gicp's RegistrationResult has no Open3D-style `fitness` field — only
    `converged`, `error` (unnormalized sum over correspondences), `num_inliers`,
    `H`, `b`. A registration between two disjoint clouds can report `converged=True`
    with `error=0.0` and `num_inliers=0` — a raw error threshold alone would accept
    that as a perfect match. Callers must gate on inlier count/ratio first, then on
    error normalized per inlier (see `align()` return value and `is_acceptable()`).
    """

    def __init__(self, max_correspondence_dist: float = 0.5,
                 downsampling_resolution: float = 0.1,
                 num_threads: int = 4,
                 registration_type: str = 'GICP'):
        self._max_corr = max_correspondence_dist
        self._voxel_size = downsampling_resolution
        self._num_threads = num_threads
        self._registration_type = registration_type

    def prepare(self, cloud: np.ndarray) -> tuple:
        """Downsample a cloud and build its KD-tree once, for reuse across aligns.

        Returns an opaque (points, tree) pair to hand back as `source_prepared` /
        `target_prepared`. Loop-closure detection scores one source against many
        candidate targets; without this, every candidate rebuilt its own tree and
        the source was re-downsampled once per candidate.
        """
        return small_gicp.preprocess_points(
            np.asarray(cloud, dtype=np.float64),
            downsampling_resolution=self._voxel_size,
            num_threads=self._num_threads)

    def align(self, source: np.ndarray, target: np.ndarray,
              initial_guess: np.ndarray = None,
              source_prepared: tuple = None,
              target_prepared: tuple = None) -> dict:
        """Align source onto target.

        Args:
            source: (N, 3) float64 point cloud to align, in its own local frame.
                Ignored when source_prepared is given.
            target: (M, 3) float64 reference point cloud, in its own local frame.
                Ignored when target_prepared is given.
            initial_guess: 4x4 SE3 transform mapping source points into the target
                frame (i.e. target ~= initial_guess @ source). Defaults to identity.
            source_prepared: cached prepare() result for source.
            target_prepared: cached prepare() result for target.

        Returns:
            dict with keys:
                T_target_source: 4x4 transform (target ~= T @ source)
                converged: bool
                error: float, unnormalized registration cost
                num_inliers: int
                num_source_points: int, size of the downsampled source cloud
                error_per_inlier: float ('inf' if num_inliers == 0)
                inlier_ratio: float in [0, 1]
        """
        if initial_guess is None:
            initial_guess = np.eye(4)

        target_down, target_tree = (target_prepared if target_prepared is not None
                                    else self.prepare(target))
        source_down, _ = (source_prepared if source_prepared is not None
                          else self.prepare(source))

        result = small_gicp.align(
            target_down, source_down, target_tree,
            init_T_target_source=initial_guess,
            registration_type=self._registration_type,
            max_correspondence_distance=self._max_corr,
            num_threads=self._num_threads)

        num_source_points = source_down.size()
        inlier_ratio = result.num_inliers / max(num_source_points, 1)
        error_per_inlier = (result.error / result.num_inliers
                            if result.num_inliers > 0 else float('inf'))

        return {
            'T_target_source': result.T_target_source,
            'converged': result.converged,
            'error': result.error,
            'num_inliers': result.num_inliers,
            'num_source_points': num_source_points,
            'error_per_inlier': error_per_inlier,
            'inlier_ratio': inlier_ratio,
        }

    @staticmethod
    def is_acceptable(result: dict, min_inlier_ratio: float = 0.3,
                       min_inlier_count: int = 50,
                       max_error_per_inlier: float = 0.05) -> bool:
        """Gate a registration result. Requires convergence AND enough inliers
        AND low per-inlier error — any one of these alone is not sufficient
        (a zero-overlap match reports converged=True, error=0.0, num_inliers=0)."""
        return (result['converged']
                and result['num_inliers'] >= min_inlier_count
                and result['inlier_ratio'] >= min_inlier_ratio
                and result['error_per_inlier'] < max_error_per_inlier)
