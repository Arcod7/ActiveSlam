#!/usr/bin/env python3
"""Reference-trajectory mission node (docs/plans/tracks/track_1_trajectory_mission.md).

Generalises drift_return_scenario.py: given a single point or a full waypoint
list (mission_waypoints), the robot follows it via the same external-goal
interface frontier_extractor already exposes (/frontier_slam/suspend +
/frontier_slam/goal, GOAL_REPUBLISH_S cadence, arrival-radius test, RELEASED
hand-back) — no new mechanism.

Arbitration with revisit_planner.py: both nodes want to drive via
/frontier_slam/goal, and there is no broker between them. Rather than a
second suspend/goal scheme, this node watches revisit_planner's own
/frontier_slam/revisit_state and yields whenever it is not 'exploring' —
holding its waypoint index, keeping suspend=true, and (to avoid two
simultaneous publishers on /frontier_slam/goal) tearing down its own goal
publisher for the duration. It resumes at the held index once revisit_state
returns to 'exploring'. It never reads /slam/dopt: revisit_planner keeps sole
ownership of the uncertainty trigger.
"""
import os
from dataclasses import dataclass
from enum import Enum

import numpy as np
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'state', 'index', 'rx', 'ry', 'rz',
    'tx', 'ty', 'tz', 'dist_m', 'revisit_state', 'event',
]

# revisit_planner.py's RevisitState.EXPLORING.value — the only state that
# does not require this node to yield. Duplicated as a literal (not imported)
# so this node has no import-time dependency on revisit_planner.py.
_REVISIT_IDLE_VALUE = 'exploring'


class MissionState(Enum):
    WAIT_ODOM = 'wait_odom'
    FOLLOWING = 'following'
    YIELDED = 'yielded'
    DONE = 'done'
    RELEASED = 'released'


@dataclass
class MissionConfig:
    arrival_radius_m: float = 2.0
    release_dwell_s: float = 5.0


def parse_mission_waypoints(flat: list) -> np.ndarray:
    """Parse a launch-settable flat [x1,y1,z1,x2,y2,z2,...] list into an
    (N, 3) array. Raises ValueError on anything ill-formed rather than
    silently truncating."""
    n = len(flat)
    if n == 0 or n % 3 != 0:
        raise ValueError(
            'mission_waypoints must be a non-empty flat list of x,y,z '
            f'triples (length a multiple of 3); got length {n}')
    return np.asarray(flat, dtype=float).reshape(-1, 3)


class TrajectoryMissionStateMachine:
    """Pure decision logic, no rclpy — directly unit-testable."""

    def __init__(self, waypoints: np.ndarray, loop: bool, cfg: MissionConfig):
        if len(waypoints) == 0:
            raise ValueError('waypoints must be non-empty')
        self.waypoints = waypoints
        self.loop = loop
        self.cfg = cfg
        self.state = MissionState.WAIT_ODOM
        self.index = 0
        self._done_since = None

    @property
    def suspended(self) -> bool:
        return self.state in (MissionState.FOLLOWING, MissionState.YIELDED,
                              MissionState.DONE)

    @property
    def holding_goal(self) -> bool:
        """True while this node should own /frontier_slam/goal."""
        return self.state in (MissionState.FOLLOWING, MissionState.DONE)

    @property
    def current_target(self) -> "np.ndarray | None":
        if self.index < len(self.waypoints):
            return self.waypoints[self.index]
        return None

    def on_odom(self) -> "str | None":
        """Call once, the first time odometry becomes available."""
        if self.state == MissionState.WAIT_ODOM:
            self.state = MissionState.FOLLOWING
            return 'STARTED'
        return None

    def tick(self, now: float, robot_xyz: np.ndarray,
             revisit_state: "str | None") -> "str | None":
        if self.state in (MissionState.WAIT_ODOM, MissionState.RELEASED):
            return None

        if self.state == MissionState.DONE:
            if now - self._done_since >= self.cfg.release_dwell_s:
                self.state = MissionState.RELEASED
                return 'RELEASED'
            return None

        yielded_now = (revisit_state is not None
                       and revisit_state != _REVISIT_IDLE_VALUE)

        if self.state == MissionState.YIELDED:
            if not yielded_now:
                self.state = MissionState.FOLLOWING
                return 'RESUMED'
            return None

        # state == FOLLOWING
        if yielded_now:
            self.state = MissionState.YIELDED
            return 'YIELDED'

        target = self.current_target
        dist = float(np.linalg.norm(np.asarray(robot_xyz)[:3] - target))
        if dist >= self.cfg.arrival_radius_m:
            return None

        if self.index == len(self.waypoints) - 1:
            if self.loop:
                self.index = 0
                return 'WAYPOINT_REACHED_LOOP'
            self.state = MissionState.DONE
            self._done_since = now
            return 'MISSION_DONE'

        self.index += 1
        return 'WAYPOINT_REACHED'


class TrajectoryMission(Node):
    TICK_HZ = 1.0
    GOAL_REPUBLISH_S = 2.0   # matches revisit_planner's / frontier_extractor's own cadence
    RELEASE_DWELL_S = 5.0

    def __init__(self):
        super().__init__('trajectory_mission')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        # No default value: an empty python-list default makes rclpy infer
        # BYTE_ARRAY instead of DOUBLE_ARRAY, which then rejects a real
        # override as a type mismatch. Declaring the bare type and treating
        # "never set" (ParameterUninitializedException) as an empty list
        # sidesteps that — and also covers a launch default_value='[]'
        # override, which resolves to the same uninitialized state.
        self.declare_parameter('mission_waypoints', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('mission_loop', False)
        self.declare_parameter('arrival_radius_m', 2.0)

        odom_topic = str(self.get_parameter('odom_topic').value)
        try:
            flat_waypoints = list(self.get_parameter('mission_waypoints').value)
        except ParameterUninitializedException:
            flat_waypoints = []
        loop = bool(self.get_parameter('mission_loop').value)
        arrival_radius_m = float(self.get_parameter('arrival_radius_m').value)

        try:
            waypoints = parse_mission_waypoints(flat_waypoints)
        except ValueError as exc:
            self.get_logger().fatal(str(exc))
            raise

        cfg = MissionConfig(arrival_radius_m=arrival_radius_m,
                            release_dwell_s=self.RELEASE_DWELL_S)
        self._sm = TrajectoryMissionStateMachine(waypoints, loop, cfg)

        self._robot_xyz: "np.ndarray | None" = None
        self._revisit_state: "str | None" = None
        self._goal_pub = None
        self._last_goal_pub_time: "float | None" = None

        self._log = open_session_log('trajectory_mission', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(String, '/frontier_slam/revisit_state',
                                 self._revisit_state_cb, 10)
        self._suspend_pub = self.create_publisher(Bool, '/frontier_slam/suspend', 1)

        self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info(
            f'trajectory_mission ready — {len(waypoints)} waypoint(s), '
            f'loop={loop} — logging to {self._log.path}')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        first = self._robot_xyz is None
        p = msg.pose.pose.position
        self._robot_xyz = np.array([p.x, p.y, p.z])
        if first:
            event = self._sm.on_odom()
            if event:
                self.get_logger().info(f'wait_odom -> {self._sm.state.value} ({event})')

    def _revisit_state_cb(self, msg: String) -> None:
        self._revisit_state = msg.data

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._robot_xyz is None:
            return

        now = self._now()
        prev_state = self._sm.state
        event = self._sm.tick(now, self._robot_xyz, self._revisit_state)

        self._suspend_pub.publish(Bool(data=self._sm.suspended))
        self._sync_goal_publisher()

        if event is not None or prev_state != self._sm.state:
            self.get_logger().info(
                f'{prev_state.value} -> {self._sm.state.value}'
                + (f' ({event})' if event else ''))

        target = self._sm.current_target
        tx = ty = tz = dist_m = float('nan')
        if self._sm.holding_goal and target is not None:
            tx, ty, tz = (float(target[0]), float(target[1]), float(target[2]))
            dist_m = float(np.linalg.norm(self._robot_xyz - target))
            if (self._last_goal_pub_time is None
                    or now - self._last_goal_pub_time >= self.GOAL_REPUBLISH_S
                    or event):
                self._publish_goal(target)
                self._last_goal_pub_time = now
        else:
            self._last_goal_pub_time = None

        self._log.write([
            now, self._sm.state.value, self._sm.index,
            float(self._robot_xyz[0]), float(self._robot_xyz[1]), float(self._robot_xyz[2]),
            tx, ty, tz, dist_m, self._revisit_state or '', event or '',
        ])

    def _sync_goal_publisher(self) -> None:
        """Create/destroy the /frontier_slam/goal publisher so this node
        only holds it while actually driving — avoids a second live
        publisher fighting revisit_planner's during a yield."""
        want = self._sm.holding_goal
        have = self._goal_pub is not None
        if want and not have:
            self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal', 1)
        elif not want and have:
            self.destroy_publisher(self._goal_pub)
            self._goal_pub = None

    def _publish_goal(self, xyz: np.ndarray) -> None:
        if self._goal_pub is None:
            return
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x = float(xyz[0])
        goal.point.y = float(xyz[1])
        goal.point.z = float(xyz[2])
        self._goal_pub.publish(goal)

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
