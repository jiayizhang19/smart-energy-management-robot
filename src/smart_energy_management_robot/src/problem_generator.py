#!/usr/bin/env python3

import random
import yaml
import os
import rclpy
import time
import subprocess
import tempfile

from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.srv import AddProblem
from plansys2_msgs.msg import ActionExecution
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


# -----------------------------------------------------------------------
# SPAWN_WAYPOINT is the waypoint name closest to the Gazebo default spawn
# position. The robot always starts here physically when Gazebo launches,
# so we always publish /initialpose here first so AMCL can localise
# correctly against the actual laser scan before any navigation begins.
# Change this if your robot spawns at a different location.
# -----------------------------------------------------------------------
SPAWN_WAYPOINT = 'entrance'


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
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.set_entity_state_cli = self.create_client(
            SetEntityState, '/gazebo/set_entity_state'
        )

        self.current_step = 0
        self.plan = []
        self.waiting_for_response = False
        self._start_nav_done = False
        self._start_nav_success = False

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
    # Gazebo teleport (best-effort)
    # ------------------------------------------------------------------

    def _teleport_robot(self, waypoint: str) -> bool:
        coords = self.waypoints_data[waypoint]
        x, y = float(coords[0]), float(coords[1])

        if not self.set_entity_state_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning(
                '/gazebo/set_entity_state not available — skipping teleport.'
            )
            return False

        state = EntityState()
        state.name = 'turtlebot3_burger'
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.0
        state.pose.orientation.w = 1.0

        req = SetEntityState.Request()
        req.state = state
        future = self.set_entity_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info(
                f'Gazebo: robot teleported to [{waypoint}] → x={x:.2f}, y={y:.2f}'
            )
            return True
        self.get_logger().warning('Gazebo teleport failed.')
        return False

    # ------------------------------------------------------------------
    # AMCL initial pose
    # ------------------------------------------------------------------

    def _publish_initial_pose(self, waypoint: str):
        """
        Publish /initialpose at the given waypoint's coordinates.
        Only reliable when the robot is physically already at that location
        so the laser scan matches the map there.
        """
        coords = self.waypoints_data[waypoint]
        x, y = float(coords[0]), float(coords[1])

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0]  = 0.25
        cov[7]  = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov

        for _ in range(10):
            self.initial_pose_pub.publish(msg)
            time.sleep(0.05)

        self.get_logger().info(
            f'AMCL: initial pose set at [{waypoint}] → x={x:.2f}, y={y:.2f}'
        )

    # ------------------------------------------------------------------
    # Navigate to waypoint — callback-based blocking wait
    # ------------------------------------------------------------------

    def _navigate_to(self, waypoint: str) -> bool:
        """
        Drive the robot to the given waypoint and block until it arrives.
        Logs distance remaining so physical movement is visible in terminal.
        """
        coords = self.waypoints_data[waypoint]
        x, y = float(coords[0]), float(coords[1])

        self.get_logger().info(
            f'Navigating to [{waypoint}] → x={x:.2f}, y={y:.2f}'
        )

        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self._start_nav_done = False
        self._start_nav_success = False

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_cb
        )
        send_future.add_done_callback(self._nav_goal_response_cb)

        while not self._start_nav_done:
            rclpy.spin_once(self, timeout_sec=0.5)

        return self._start_nav_success

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self._start_nav_done = True
            self._start_nav_success = False
            return
        self.get_logger().info('Navigation goal accepted.')
        goal_handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        self._start_nav_done = True
        self._start_nav_success = True
        self.get_logger().info('Navigation complete — robot has arrived.')

    def _nav_feedback_cb(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'  Distance remaining: {distance:.2f} m')

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
        self.waiting_for_response = True

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
                self.waiting_for_response = False
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
        shuffled = random.sample(self.waypoints, len(self.waypoints))
        start_wp    = shuffled[0]
        critical_wp = shuffled[1]
        high_wp     = shuffled[2]

        self.get_logger().info(f'Start waypoint:    {start_wp}')
        self.get_logger().info(f'Critical waypoint: {critical_wp}')
        self.get_logger().info(f'High waypoint:     {high_wp}')

        # ------------------------------------------------------------------
        # LOCALISATION FIRST
        # The robot always physically spawns at SPAWN_WAYPOINT (entrance).
        # Publish /initialpose there so AMCL localises against the actual
        # laser scan. This must happen before any navigation.
        # ------------------------------------------------------------------
        self.get_logger().info(
            f'Publishing spawn pose at [{SPAWN_WAYPOINT}] so AMCL can localise...'
        )
        self._publish_initial_pose(SPAWN_WAYPOINT)
        self.get_logger().info('Waiting for AMCL to converge at spawn point...')
        time.sleep(5.0)  # give AMCL time to match laser scan to map

        # ------------------------------------------------------------------
        # NAVIGATE TO RANDOM START
        # If start_wp differs from spawn, drive robot there now.
        # Teleport is attempted first (instant); Nav2 is used as fallback.
        # ------------------------------------------------------------------
        if start_wp != SPAWN_WAYPOINT:
            self.get_logger().info(
                f'Start waypoint [{start_wp}] differs from spawn [{SPAWN_WAYPOINT}]'
                f' — moving robot to start...'
            )

            # Try Gazebo teleport first
            teleported = self._teleport_robot(start_wp)

            if teleported:
                # Teleport succeeded — update AMCL to new position
                time.sleep(1.0)
                self._publish_initial_pose(start_wp)
                time.sleep(3.0)
            else:
                # Teleport unavailable — physically drive there via Nav2.
                # AMCL is already localised at spawn so Nav2 can plan correctly.
                arrived = self._navigate_to(start_wp)
                if not arrived:
                    self.get_logger().error(
                        'Failed to reach start waypoint — aborting.'
                    )
                    return
                # Re-publish pose after arrival
                self._publish_initial_pose(start_wp)
                time.sleep(1.0)
        else:
            self.get_logger().info(
                f'Start waypoint is spawn point [{SPAWN_WAYPOINT}] — no movement needed.'
            )

        # ------------------------------------------------------------------
        # GENERATE AND EXECUTE PLAN
        # Robot is now physically at start_wp and AMCL is localised there.
        # ------------------------------------------------------------------
        problem = self._build_problem(start_wp, critical_wp, high_wp)
        self.get_logger().info('Generating plan with POPF...')
        self.plan = self._call_popf(problem)

        if not self.plan:
            self.get_logger().error('POPF could not generate a plan')
            return

        self.get_logger().info(f'Plan with {len(self.plan)} steps:')
        for i, (action, args) in enumerate(self.plan):
            self.get_logger().info(f'  {i+1}. {action} {args}')

        self.add_problem_cli.wait_for_service(timeout_sec=10.0)
        req = AddProblem.Request()
        req.problem = problem
        future = self.add_problem_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Problem added to PlanSys2')

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