"""Score revisit candidates on the distance the vehicle must actually swim.

`select_revisit_target` scores travel as straight-line distance. Measured on the
a007 batch, that is what sends the vehicle to targets it cannot reach: the worst
episode picked the *nearest* candidate by Euclidean score, 18.8 m, whose A* route
round the hull was 65.7 m — 126 s at the measured 0.52 m/s, against a 120 s
transit budget. Five of seven episodes never arrived.

One Dijkstra from the robot over the cost grid the planner already builds labels
every cell with its true cost-to-reach, so each candidate is then an array
lookup. That runs once per trigger (~every two minutes), not per tick.

Deliberately a weighted cost and not a reachability veto: one episode swam 33.6 m,
ended 4.4 m short, and still collected constraints the whole way, so a long route
is expensive rather than worthless.

Unknown cells cost the same as free (`path_planner.py:16`), so the wavefront runs
straight through unexplored space. That is what separates "no route known yet"
from "the hull is in the way" instead of compromising between them.
"""
from __future__ import annotations

import numpy as np

# 8-connectivity, with the diagonal step length that goes with each offset.
_OFFSETS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 2.0 ** 0.5), (-1, 1, 2.0 ** 0.5),
            (1, -1, 2.0 ** 0.5), (1, 1, 2.0 ** 0.5)]


def _world_to_cell(cg, xy):
    col = int((xy[0] - cg.ox) / cg.res)
    row = int((xy[1] - cg.oy) / cg.res)
    return row, col


def travel_cost_field(cg, robot_xy) -> "np.ndarray | None":
    """Metres-equivalent cost to reach every cell from the robot, or None.

    inf marks cells no route reaches. Entering a cell costs that cell's zone
    multiplier times the step length, matching what A* charges, so the field is
    comparable with the routes the planner will actually produce.
    """
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra

    cost = np.asarray(cg.cost_grid, dtype=float)
    h, w = cost.shape
    r0, c0 = _world_to_cell(cg, robot_xy)
    if not (0 <= r0 < h and 0 <= c0 < w) or not np.isfinite(cost[r0, c0]):
        return None

    passable = np.isfinite(cost)
    idx = np.arange(h * w).reshape(h, w)
    rows, cols, vals = [], [], []
    for dr, dc, step in _OFFSETS:
        src = passable[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
        dst = passable[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)]
        both = src & dst
        if not both.any():
            continue
        si = idx[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)][both]
        di = idx[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)][both]
        wgt = cost[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)][both]
        rows.append(si)
        cols.append(di)
        vals.append(wgt * step * cg.res)
    if not rows:
        return None
    g = coo_matrix((np.concatenate(vals),
                    (np.concatenate(rows), np.concatenate(cols))),
                   shape=(h * w, h * w)).tocsr()
    d = dijkstra(g, directed=True, indices=r0 * w + c0)
    return d.reshape(h, w)


def _unit_scale(values: np.ndarray) -> np.ndarray:
    peak = float(np.max(values)) if values.size else 0.0
    return values / peak if peak > 0.0 else np.zeros(values.shape)


def select_revisit_target_geodesic(kf_xyz, robot_xy, cost_grid, *,
                                   min_index_gap: int, candidate_radius_m: float,
                                   min_target_dist_m: float, w_density: float,
                                   w_travel: float, w_age: float = 0.0,
                                   max_travel_m: float = float('inf')):
    """Pick a keyframe index to revisit, scoring travel geodesically.

    Returns (index, info) or (None, info). `info['fallback']` is True when no
    usable cost field was available and the caller should fall back to the
    Euclidean selector — that is today's behaviour, so this degrades safely.
    """
    kf = np.asarray(kf_xyz, dtype=float)
    n = len(kf)
    info = {'fallback': False, 'n_eligible': 0, 'n_reachable': 0}
    if n == 0:
        return None, info

    idx = np.arange(n)
    eligible = idx <= (n - 1 - min_index_gap)
    if not np.any(eligible):
        return None, info
    e_idx = idx[eligible]
    e_xy = kf[eligible][:, :2]
    robot_xy = np.asarray(robot_xy, dtype=float)[:2]
    info['n_eligible'] = int(len(e_idx))

    field = travel_cost_field(cost_grid, robot_xy) if cost_grid is not None else None
    if field is None:
        info['fallback'] = True
        return None, info

    h, w = field.shape
    rc = np.stack([((e_xy[:, 1] - cost_grid.oy) / cost_grid.res).astype(int),
                   ((e_xy[:, 0] - cost_grid.ox) / cost_grid.res).astype(int)], axis=1)
    inside = ((rc[:, 0] >= 0) & (rc[:, 0] < h) & (rc[:, 1] >= 0) & (rc[:, 1] < w))
    travel = np.full(len(e_idx), np.inf)
    travel[inside] = field[rc[inside, 0], rc[inside, 1]]

    straight = np.linalg.norm(e_xy - robot_xy, axis=1)
    keep = np.isfinite(travel) & (travel <= max_travel_m) & (straight >= min_target_dist_m)
    info['n_reachable'] = int(np.sum(np.isfinite(travel)))
    if not np.any(keep):
        info['fallback'] = True
        return None, info

    pair = np.linalg.norm(e_xy[:, None, :] - e_xy[None, :, :], axis=2)
    density = (np.sum(pair < candidate_radius_m, axis=1) - 1).astype(float)

    d = _unit_scale(density[keep])
    t = _unit_scale(travel[keep])
    # Age as index gap behind the newest keyframe: among candidates at similar
    # range, prefer the one whose neighbourhood has gone unobserved longest.
    age = _unit_scale(((n - 1) - e_idx[keep]).astype(float))
    score = w_density * d - w_travel * t + w_age * age

    pick = int(np.argmax(score))
    info.update(travel_m=float(travel[keep][pick]),
                straight_m=float(straight[keep][pick]),
                tortuosity=float(travel[keep][pick] / max(straight[keep][pick], 1e-6)))
    return int(e_idx[keep][pick]), info
