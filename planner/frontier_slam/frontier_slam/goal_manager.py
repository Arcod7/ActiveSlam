"""Stateful goal selection from frontier clusters.

Goal commitment model — a goal changes only in two cases:
  1. Arrived   — robot is within goal_radius of the committed position.
  2. Stuck     — no progress toward the goal for stuck_timeout seconds.

Score (distance / cluster_size) is used only when picking a NEW goal, never
to preempt a currently committed one.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class GoalSelection:
    """Result of one GoalManager.select() call."""
    gx: float
    gy: float
    stuck_pct: int        # 0–100, percent of stuck timeout elapsed on current goal
    # '', 'STUCK_BLACKLIST', 'ALL_BLACKLISTED', 'OUTSIDE_SURVEY_AREA'
    event: str = ''
    # Populated only when event == 'STUCK_BLACKLIST', for user-facing logging.
    stuck_goal: tuple | None = None     # (x, y, elapsed_s, progress_m)


class GoalManager:
    def __init__(self, *,
                 min_explore_dist: float = 3.0,
                 goal_vanish_dist: float = 3.0,
                 goal_radius: float = 2.0,
                 stuck_timeout: float = 30.0,
                 stuck_min_progress: float = 0.5,
                 blacklist_duration: float = 60.0,
                 arrival_blacklist_duration: float = 20.0,
                 survey_center: tuple | None = None,
                 survey_radius: float = 0.0,
                 min_goal_separation: float = 0.0):
        self.min_explore_dist           = min_explore_dist
        self.goal_vanish_dist           = goal_vanish_dist
        self.goal_radius                = goal_radius
        self.stuck_timeout              = stuck_timeout
        self.stuck_min_progress         = stuck_min_progress
        self.blacklist_duration         = blacklist_duration
        self.arrival_blacklist_duration = arrival_blacklist_duration
        # Survey working area: frontier goals outside it are not candidates,
        # so exploration stays on the structure instead of following open
        # water outward without bound. radius <= 0 disables it entirely, which
        # is the default -- every existing run behaves exactly as before.
        self.survey_center              = survey_center
        self.survey_radius              = survey_radius
        # Consecutive goals must be at least this far apart. Without it the
        # planner can re-pick a cluster a metre from the one it just reached,
        # so the vehicle shuffles on the spot instead of moving on. Dropped
        # when nothing else qualifies, so a lone remaining frontier is still
        # reachable. 0 disables.
        self.min_goal_separation        = min_goal_separation

        self._committed: np.ndarray | None = None
        self._committed_time: float = 0.0
        self._last_goal: np.ndarray | None = None    # the goal before this one
        # Sliding-window stuck detection: resets when robot gets closer to goal OR
        # when robot has physically moved (displacement ≥ stuck_min_progress from
        # last-reset position).  The displacement check prevents false STUCK during
        # large A* detour arcs where goal-distance temporarily increases.
        self._closest_dist: float = float('inf')
        self._closest_t: float = 0.0
        self._closest_ref_pos: np.ndarray | None = None
        self._blacklist: list = []   # [(wx, wy, expiry_time)]

    # ---- queries
    @property
    def blacklist_size(self) -> int:
        return len(self._blacklist)

    def is_blacklisted(self, wx: float, wy: float) -> bool:
        return any(
            np.hypot(wx - x, wy - y) < self.goal_vanish_dist
            for x, y, _ in self._blacklist
        )

    # ---- main entry point
    def inside_survey_area(self, wx: float, wy: float) -> bool:
        """Whether a goal lies within the configured working area."""
        if self.survey_radius <= 0.0 or self.survey_center is None:
            return True
        return float(np.hypot(wx - self.survey_center[0],
                              wy - self.survey_center[1])) <= self.survey_radius

    def select(self, clusters, robot_xy: np.ndarray, now: float):
        """Pick the next goal from the cluster list.

        Returns None when all candidates are within MIN_EXPLORE_DIST.
        Returns a GoalSelection with event='ALL_BLACKLISTED' when every
        remaining candidate is blacklisted, or 'OUTSIDE_SURVEY_AREA' when
        every candidate lies beyond the survey radius -- the working area has
        been explored, which is a finished mission rather than a fault.
        """
        self._blacklist = [(x, y, t) for x, y, t in self._blacklist if t > now]

        candidates = [c for c in clusters if c.distance >= self.min_explore_dist]
        if not candidates:
            return None

        # Bound before blacklisting: a goal outside the area is not a failed
        # goal, so it must not consume a blacklist slot or trigger STUCK.
        in_area = [c for c in candidates if self.inside_survey_area(c.wx, c.wy)]
        if candidates and not in_area:
            if self._committed is not None:
                self._last_goal = self._committed.copy()
            self._committed = None
            return GoalSelection(float('nan'), float('nan'), 0,
                                 'OUTSIDE_SURVEY_AREA')
        candidates = in_area

        candidates = [c for c in candidates if not self.is_blacklisted(c.wx, c.wy)]
        if not candidates:
            return GoalSelection(float('nan'), float('nan'), 0, 'ALL_BLACKLISTED')

        # Score = distance / size: prefer large clusters and close ones equally.
        candidates.sort(key=lambda c: c.distance / c.size)

        stuck_info = self._check_and_blacklist_if_stuck(candidates, robot_xy, now)
        event = 'STUCK_BLACKLIST' if stuck_info else ''
        if stuck_info:
            candidates = [c for c in candidates if not self.is_blacklisted(c.wx, c.wy)]
            if not candidates:
                return GoalSelection(float('nan'), float('nan'), 0, event, stuck_info)

        gx, gy = self._pick_committed(candidates, robot_xy, now)
        time_no_progress = now - self._closest_t
        stuck_pct = min(100, int(time_no_progress / self.stuck_timeout * 100))
        return GoalSelection(gx, gy, stuck_pct, event, stuck_info)

    # ---- internals
    def _check_and_blacklist_if_stuck(self, candidates, robot_xy, now):
        if self._committed is None:
            return None
        cur_dist = np.hypot(self._committed[0] - robot_xy[0],
                            self._committed[1] - robot_xy[1])

        # Update sliding window: reset when robot gets closer to goal …
        if cur_dist < self._closest_dist - self.stuck_min_progress:
            self._closest_dist    = cur_dist
            self._closest_t       = now
            self._closest_ref_pos = robot_xy.copy()
        # … or when robot has physically moved (catches A* detour arcs where
        # goal-distance temporarily increases while navigating around obstacles).
        elif (self._closest_ref_pos is not None and
              np.hypot(robot_xy[0] - self._closest_ref_pos[0],
                       robot_xy[1] - self._closest_ref_pos[1])
              >= self.stuck_min_progress):
            self._closest_t       = now
            self._closest_ref_pos = robot_xy.copy()

        if now - self._closest_t < self.stuck_timeout:
            return None

        elapsed = now - self._committed_time
        info = (float(self._committed[0]), float(self._committed[1]),
                float(elapsed), float(self._closest_dist))
        self._blacklist.append((self._committed[0], self._committed[1],
                                now + self.blacklist_duration))
        self._last_goal = self._committed.copy()
        self._committed = None
        return info

    def _fresh_pick(self, candidates):
        """Best-scored candidate that is far enough from the previous goal.

        Candidates arrive sorted by score, so this is the first one clearing
        the separation. Falls back to the outright best when none does, which
        is what keeps a single remaining frontier reachable."""
        # The goal being left is the committed one while it still exists, and
        # _last_goal once it has been cleared (arrival, stuck, out of area).
        reference = (self._committed if self._committed is not None
                     else self._last_goal)
        if reference is not None and self.min_goal_separation > 0.0:
            spread = [c for c in candidates
                      if np.hypot(c.wx - reference[0], c.wy - reference[1])
                      >= self.min_goal_separation]
            if spread:
                return spread[0]
        return candidates[0]

    def _pick_committed(self, candidates, robot_xy, now):
        """Return the goal to send.  Never switches away from a committed goal
        except on arrival — STUCK is the only other exit, handled upstream."""
        best = self._fresh_pick(candidates)   # used only when picking a fresh goal

        if self._committed is None:
            self._commit(best.wx, best.wy, robot_xy, now)
            return best.wx, best.wy

        # Track map-drift of the committed cluster.
        near_old = [c for c in candidates
                    if np.hypot(c.wx - self._committed[0], c.wy - self._committed[1])
                    < self.goal_vanish_dist]
        if near_old:
            drift = min(near_old, key=lambda c: np.hypot(c.wx - self._committed[0],
                                                         c.wy - self._committed[1]))
            self._committed = np.array([drift.wx, drift.wy])
            return drift.wx, drift.wy

        # Cluster vanished — check whether the robot has arrived.
        cur_dist = np.hypot(robot_xy[0] - self._committed[0],
                            robot_xy[1] - self._committed[1])
        if cur_dist <= self.goal_radius:
            # Arrived. Blacklist briefly so the robot doesn't immediately re-pick it.
            self._blacklist.append((self._committed[0], self._committed[1],
                                    now + self.arrival_blacklist_duration))
            self._commit(best.wx, best.wy, robot_xy, now)
            return best.wx, best.wy

        # Cluster gone but robot hasn't arrived yet — keep heading to the last
        # known position.  STUCK will fire if progress stalls.
        return float(self._committed[0]), float(self._committed[1])

    def mark_unreachable(self, goal_xy: np.ndarray, now: float) -> None:
        """Blacklist a goal that A* consistently cannot path-plan to."""
        wx, wy = float(goal_xy[0]), float(goal_xy[1])
        self._blacklist.append((wx, wy, now + self.blacklist_duration))
        if (self._committed is not None and
                np.hypot(self._committed[0] - wx, self._committed[1] - wy)
                < self.goal_vanish_dist):
            self._committed       = None
            self._closest_dist    = float('inf')
            self._closest_t       = 0.0
            self._closest_ref_pos = None

    def _commit(self, gx: float, gy: float, robot_xy: np.ndarray, now: float) -> None:
        # Remember the outgoing goal before it is replaced: the separation
        # rule is stated against the goal the vehicle just came from.
        if self._committed is not None:
            self._last_goal = self._committed.copy()
        self._committed       = np.array([gx, gy])
        self._committed_time  = now
        d = float(np.hypot(gx - robot_xy[0], gy - robot_xy[1]))
        self._closest_dist    = d
        self._closest_t       = now
        self._closest_ref_pos = robot_xy.copy()
