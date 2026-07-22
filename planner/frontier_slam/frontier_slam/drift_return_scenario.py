#!/usr/bin/env python3
"""Scripted drift-and-return scenario node (docs/plans/plan.md T1.1).

A fixed, non-random waypoint sequence: leave the start position far enough
to accumulate visible dead-reckoning drift under noise_profile:=realistic,
then return to it so an opportunistic loop closure (pose_graph.py) has a
chance to fire. Reuses frontier_extractor's own external-goal interface
(/frontier_slam/suspend + /frontier_slam/goal) — the same pattern
revisit_planner.py uses — so no planner or controller changes are needed.
Reproducibility comes from the launch's existing noise_seed argument; the
waypoint sequence itself is deterministic.

After the return leg, the correction from a loop closure lands within a
few seconds (confirmed on a live run: abs_error dropped 90%+ within ~2s of
the closure) — RELEASE_DWELL_S just gives one redetect cycle a moment to
settle before handing goal publication back to frontier_extractor, rather
than holding the robot in place indefinitely.
"""
import os
from enum import Enum

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = ['t_ros', 'state', 'rx', 'ry', 'tx', 'ty', 'dist_m', 'event']


class ScenarioState(Enum):
    WAIT_ODOM = 'wait_odom'
    OUTBOUND = 'outbound'
    RETURN = 'return'
    DONE = 'done'
    RELEASED = 'released'


class DriftReturnScenario(Node):
    GOAL_REPUBLISH_S = 2.0   # matches revisit_planner's own cadence
    TICK_HZ = 1.0
    RELEASE_DWELL_S = 12.0   # hold at start this long after arrival, then resume exploration

    def __init__(self):
        super().__init__('drift_return_scenario')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('out_dx', 15.0)
        self.declare_parameter('out_dy', 0.0)
        self.declare_parameter('arrival_radius_m', 2.0)

        odom_topic = str(self.get_parameter('odom_topic').value)
        self._out_dx = float(self.get_parameter('out_dx').value)
        self._out_dy = float(self.get_parameter('out_dy').value)
        self._arrival_radius_m = float(self.get_parameter('arrival_radius_m').value)

        self._state = ScenarioState.WAIT_ODOM
        self._start_xy: np.ndarray | None = None
        self._target_out_xy: np.ndarray | None = None
        self._robot_xyz: np.ndarray | None = None
        self._last_goal_pub_time: float | None = None
        self._done_since: float | None = None

        self._log = open_session_log('drift_return', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self._suspend_pub = self.create_publisher(Bool, '/frontier_slam/suspend', 1)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal', 1)

        self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info(f'drift_return_scenario ready — logging to {self._log.path}')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_xyz = np.array([p.x, p.y, p.z])
        if self._state == ScenarioState.WAIT_ODOM:
            self._start_xy = self._robot_xyz[:2].copy()
            self._target_out_xy = self._start_xy + np.array([self._out_dx, self._out_dy])
            self._state = ScenarioState.OUTBOUND
            self._suspend_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'Start captured at ({self._start_xy[0]:.1f},{self._start_xy[1]:.1f}) — '
                f'heading out to ({self._target_out_xy[0]:.1f},{self._target_out_xy[1]:.1f})'
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._robot_xyz is None or self._state == ScenarioState.WAIT_ODOM:
            return
        if self._state == ScenarioState.RELEASED:
            return   # frontier_extractor owns goal publication again — stay out of the way

        now = self._now()
        self._suspend_pub.publish(Bool(data=True))

        target_xy = (self._target_out_xy if self._state == ScenarioState.OUTBOUND
                     else self._start_xy)
        dist = float(np.hypot(self._robot_xyz[0] - target_xy[0],
                              self._robot_xyz[1] - target_xy[1]))

        event = ''
        if self._state == ScenarioState.OUTBOUND and dist < self._arrival_radius_m:
            self._state = ScenarioState.RETURN
            event = 'OUTBOUND_ARRIVED'
            target_xy = self._start_xy
            dist = float(np.hypot(self._robot_xyz[0] - target_xy[0],
                                  self._robot_xyz[1] - target_xy[1]))
            self.get_logger().info('Outbound leg complete — returning to start')
        elif self._state == ScenarioState.RETURN and dist < self._arrival_radius_m:
            self._state = ScenarioState.DONE
            self._done_since = now
            event = 'RETURN_ARRIVED'
            self.get_logger().info(
                f'Return leg complete — holding {self.RELEASE_DWELL_S:.0f}s for the '
                'correction to land, then resuming exploration'
            )
        elif (self._state == ScenarioState.DONE
              and now - self._done_since >= self.RELEASE_DWELL_S):
            self._state = ScenarioState.RELEASED
            event = 'RELEASED'
            self._suspend_pub.publish(Bool(data=False))
            self.get_logger().info(
                'Releasing control back to frontier_extractor — resuming exploration'
            )
            self._log.write([now, self._state.value,
                             float(self._robot_xyz[0]), float(self._robot_xyz[1]),
                             float(target_xy[0]), float(target_xy[1]), dist, event])
            return

        if (self._last_goal_pub_time is None
                or now - self._last_goal_pub_time >= self.GOAL_REPUBLISH_S
                or event):
            self._publish_goal(target_xy)
            self._last_goal_pub_time = now

        self._log.write([now, self._state.value,
                         float(self._robot_xyz[0]), float(self._robot_xyz[1]),
                         float(target_xy[0]), float(target_xy[1]), dist, event])

    def _publish_goal(self, target_xy: np.ndarray) -> None:
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x = float(target_xy[0])
        goal.point.y = float(target_xy[1])
        goal.point.z = float(self._robot_xyz[2])
        self._goal_pub.publish(goal)

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriftReturnScenario()
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
