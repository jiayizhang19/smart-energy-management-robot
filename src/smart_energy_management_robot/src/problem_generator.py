#!/usr/bin/env python3

import random
import yaml
import os
import rclpy
import time
import subprocess
import tempfile

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.srv import AddProblem
from plansys2_msgs.msg import ActionExecution
from geometry_msgs.msg import PoseWithCovarianceStamped


# -----------------------------------------------------------------------
# Fixed start waypoint — must match where the robot physically spawns
# in Gazebo. 'entrance' is closest to the default Gazebo spawn (0, 0).
# Critical and high energy waypoints are still randomly assigned.
# -----------------------------------------------------------------------
START_WAYPOINT = 'entrance'


class ProblemGenerator(Node):

    def __init__(self):
        super().__init__('problem_generator')
        self.waypoints_data = self._load_waypoints()
        self.waypoints = list(self.waypoints_data.keys())

        self.domain_path = os.path.join(
            get_package_share_directory('smart_energy_management_robot'),
            'pddl', 'domain.pddl'
        )

        self.add_problem_cli = self.create_client(
            AddProblem, 'problem_expert/add_problem'
        )
        self.action_pub = self.create_publisher(
            ActionExecution, 'actions_hub', 10
        )
        self.action_sub = self.create_subscription(
            ActionExecution, 'actions_hub',
            self._action_response_cb, 10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        self.current_step = 0
        self.plan = []

    # ------------------------------------------------------------------
    # Data loading
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        pkg = get_package_share_directory('smart_energy_management_robot')
        path = os.path.join(pkg, 'config', 'waypoints.yaml')
        with open(path) as f:
            data = yaml.safe_load(f)
        return data['waypoints']

    # ------------------------------------------------------------------
    # AMCL initial pose — published at the fixed spawn location so the
    # laser scan matches the map and AMCL converges immediately.
    # No manual 2D Pose Estimate in RViz needed.
    # ------------------------------------------------------------------

    def _publish_initial_pose(self):
        coords = self.waypoints_data[START_WAYPOINT]
        x, y = float(coords[0]), float(coords[1])

        timeout_s = 5.0
        start = time.time()
        while self.initial_pose_pub.get_subscription_count() == 0:
            if time.time() - start > timeout_s:
                self.get_logger().warning(
                    'No AMCL subscriber for /initialpose after 5s; publishing anyway'
                )
                break
            time.sleep(0.1)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Standard RViz 2D Pose Estimate covariance defaults
        cov = [0.0] * 36
        cov[0]  = 0.25   # x variance
        cov[7]  = 0.25   # y variance
        cov[35] = 0.07   # yaw variance
        msg.pose.covariance = cov

        # Publish multiple times — topic is not latched
        for _ in range(10):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info(
            f'AMCL: initial pose set at [{START_WAYPOINT}] → x={x:.2f}, y={y:.2f}'
        )

    # ------------------------------------------------------------------
    # PDDL problem
    # ------------------------------------------------------------------

    def _build_problem(self, start_wp, critical_wp, high_wp):
        objects = '  ' + ' '.join(self.waypoints) + ' - waypoint'
        init_lines = [f'  (robot_at {start_wp})']
        init_lines.append(f'  (visited {start_wp})')
        init_lines.append('  (critical_energy_active)')
        init_lines.append('  (high_energy_active)')
        init_lines.append(f'  (is_critical_wp {critical_wp})')
        init_lines.append(f'  (is_high_wp {high_wp})')
        for wp1 in self.waypoints:
            for wp2 in self.waypoints:
                if wp1 != wp2:
                    init_lines.append(f'  (connected {wp1} {wp2})')

        goal_lines = ['  (and']
        for wp in self.waypoints:
            if wp != start_wp:
                goal_lines.append(f'    (visited {wp})')
        goal_lines.append('    (priorities_cleared)')
        goal_lines.append('  )')

        return (
            '(define (problem energy_problem)\n'
            '  (:domain energy_management)\n'
            '  (:objects\n'
            f'{objects}\n'
            '  )\n'
            '  (:init\n'
            + '\n'.join(init_lines) + '\n'
            '  )\n'
            '  (:goal\n'
            + '\n'.join(goal_lines) + '\n'
            '  )\n'
            ')'
        )

    # ------------------------------------------------------------------
    # POPF
    # ------------------------------------------------------------------

    def _call_popf(self, problem_str):
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.pddl', delete=False
        ) as f:
            f.write(problem_str)
            problem_path = f.name

        try:
            result = subprocess.run(
                ['/opt/ros/humble/lib/popf/popf',
                 self.domain_path, problem_path],
                capture_output=True, text=True, timeout=30
            )
            plan = []
            for line in result.stdout.split('\n'):
                if ': (' in line and '[' in line:
                    action_part = line.split('(')[1].split(')')[0].strip()
                    parts = action_part.split()
                    plan.append((parts[0], parts[1:]))
            return plan
        finally:
            os.unlink(problem_path)

    # ------------------------------------------------------------------
    # Action dispatching
    # ------------------------------------------------------------------

    def _dispatch_next_action(self):
        if self.current_step >= len(self.plan):
            self.get_logger().info('All actions completed successfully!')
            return

        action_name, args = self.plan[self.current_step]
        self.get_logger().info(
            f'Dispatching step {self.current_step + 1}/{len(self.plan)}: '
            f'{action_name} {args}'
        )

        msg = ActionExecution()
        msg.type = ActionExecution.REQUEST
        msg.node_id = 'problem_generator'
        msg.action = action_name
        msg.arguments = args
        msg.success = False
        msg.completion = 0.0
        msg.status = 'requested'
        self.action_pub.publish(msg)

    def _action_response_cb(self, msg):
        if msg.type == ActionExecution.REQUEST:
            return
        if msg.node_id == 'problem_generator':
            return

        if msg.type == ActionExecution.FINISH:
            if msg.success:
                self.get_logger().info(
                    f'Action {msg.action} completed: {msg.status}'
                )
                self.current_step += 1
                self._dispatch_next_action()
            else:
                self.get_logger().error(
                    f'Action {msg.action} failed: {msg.status}'
                )
        elif msg.type == ActionExecution.FEEDBACK:
            self.get_logger().info(
                f'Action {msg.action} feedback: {msg.status} '
                f'({msg.completion * 100:.0f}%)'
            )

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def setup_and_trigger(self):
        # Fixed start, random critical and high waypoints
        remaining = [wp for wp in self.waypoints if wp != START_WAYPOINT]
        shuffled = random.sample(remaining, len(remaining))
        critical_wp = shuffled[0]
        high_wp     = shuffled[1]

        self.get_logger().info(f'Start waypoint:    {START_WAYPOINT} (fixed)')
        self.get_logger().info(f'Critical waypoint: {critical_wp}')
        self.get_logger().info(f'High waypoint:     {high_wp}')
        self.get_logger().info(
            f'Run summary: start={START_WAYPOINT}, critical={critical_wp}, high={high_wp}'
        )

        # Publish initial pose at spawn location so AMCL localises correctly.
        # Robot is physically here — laser scan will match the map immediately.
        self.get_logger().info('Publishing initial pose to AMCL...')
        self._publish_initial_pose()

        # Give AMCL time to converge before Nav2 starts planning paths
        self.get_logger().info('Waiting for AMCL to converge...')
        time.sleep(5.0)

        # Build and solve PDDL problem
        problem = self._build_problem(START_WAYPOINT, critical_wp, high_wp)
        self.get_logger().info('Generating plan with POPF...')
        self.plan = self._call_popf(problem)

        if not self.plan:
            self.get_logger().error('POPF could not generate a plan')
            return

        self.get_logger().info(f'Plan with {len(self.plan)} steps:')
        for i, (action, args) in enumerate(self.plan):
            self.get_logger().info(f'  {i+1}. {action} {args}')
        sequence = ' -> '.join(
            f'{i+1}:{action}({" ".join(args)})'
            for i, (action, args) in enumerate(self.plan)
        )
        self.get_logger().info(f'Plan sequence: {sequence}')

        # Register with PlanSys2
        self.add_problem_cli.wait_for_service(timeout_sec=10.0)
        req = AddProblem.Request()
        req.problem = problem
        future = self.add_problem_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Problem added to PlanSys2')

        # Start execution
        self.current_step = 0
        self._dispatch_next_action()


def main():
    rclpy.init()
    node = ProblemGenerator()
    node.get_logger().info('Waiting for PlanSys2 and Nav2...')
    time.sleep(3.0)
    node.setup_and_trigger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()