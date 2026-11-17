"""Z-band for octomap_server's planning-map projection.

octomap_server's occupancy_min_z/occupancy_max_z bound which Z range gets
collapsed into each /projected_map cell. Left unset (the default), the whole
water column — seabed to surface — is flattened into one cell per XY
position: a "Z-sheet" with no depth information, so a frontier/A* decision at
the robot's actual depth is really made against a mash-up of everything
above and below it.

octomap_z_band_params() narrows that to a band centred on the robot's
commanded cruise depth. Its half-thickness is the caller's
projected_map_band_m — the same number the TSDF mapper and the frontier
extractor band their own projections by, so every backend plans against a
slice of the same size. ROBOT_HEIGHT_M is only the floor under it: depth-hold
isn't precise and the hull itself has vertical extent, so a band thinner than
the vehicle is a single-Z assumption in disguise.

The band is only applied when depth is an explicit, known value — depth < 0
means "auto-lock from the first odometry reading" (frontier_slam.launch.py's
`depth` argument), which is unresolved at launch time, so filtering to an
unknown Z would be worse than not filtering at all. That case keeps today's
full-column projection.
"""

# BlueROV2 Heavy overall height (Blue Robotics datasheet: 254 mm), rounded
# for headroom against depth-hold imprecision.
ROBOT_HEIGHT_M = 0.25


def octomap_z_band_params(depth_m: float, band_m: float = ROBOT_HEIGHT_M) -> dict:
    """occupancy_min_z/max_z spanning 2x band_m centred on depth_m.

    Returns {} (full-column projection) when depth_m < 0.
    """
    if depth_m < 0.0:
        return {}
    half = max(abs(band_m), ROBOT_HEIGHT_M)
    return {
        'occupancy_min_z': depth_m - half,
        'occupancy_max_z': depth_m + half,
    }
