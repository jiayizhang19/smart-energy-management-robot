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


class ProblemGenerator(Node):

    def __init__(self):
        super().__init__('problem_generator')
        self.waypoints = self._load_waypoint_names()
        self.domain_path = os.path.join(
            get_package_share_directory('smart_energy_management_robot'),
            'pddl', 'domain.pddl'
        )
        self.add_problem_cli = self.create_client(
            AddProblem, 'problem_expert/add_problem'
        )
        # publish actions directly to actions_hub
        self.action_pub = self.create_publisher(
            ActionExecution, 'actions_hub', 10
        )
        # subscribe to get responses from executor
        self.action_sub = self.create_subscription(
            ActionExecution, 'actions_hub',
            self._action_response_cb, 10
        )
        self.current_step = 0
        self.plan = []
        self.waiting_for_response = False

    def _load_waypoint_names(self):
        pkg = get_package_share_directory('smart_energy_management_robot')
        path = os.path.join(pkg, 'config', 'waypoints.yaml')
        with open(path) as f:
            data = yaml.safe_load(f)
        return list(data['waypoints'].keys())

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

    def _call_popf(self, problem_str):
        """Call POPF directly, return list of (action, [args]) tuples."""
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
                # plan lines look like: 0.000: (visit_critical a b)  [0.001]
                if ': (' in line and '[' in line:
                    # extract action part between ( and )
                    action_part = line.split('(')[1].split(')')[0].strip()
                    parts = action_part.split()
                    action_name = parts[0]
                    args = parts[1:]
                    plan.append((action_name, args))
            return plan
        finally:
            os.unlink(problem_path)

    def _dispatch_next_action(self):
        """Dispatch the next action in the plan to the executor."""
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
        """Handle responses from the action executor."""
        # ignore our own requests
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
                # dispatch next action
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

    def setup_and_trigger(self):
        shuffled = random.sample(self.waypoints, len(self.waypoints))
        start_wp    = shuffled[0]
        critical_wp = shuffled[1]
        high_wp     = shuffled[2]

        self.get_logger().info(f'Start waypoint:    {start_wp}')
        self.get_logger().info(f'Critical waypoint: {critical_wp}')
        self.get_logger().info(f'High waypoint:     {high_wp}')

        problem = self._build_problem(start_wp, critical_wp, high_wp)

        # verify with POPF
        self.get_logger().info('Generating plan with POPF...')
        self.plan = self._call_popf(problem)

        if not self.plan:
            self.get_logger().error('POPF could not generate a plan')
            return

        self.get_logger().info(f'Plan with {len(self.plan)} steps:')
        for i, (action, args) in enumerate(self.plan):
            self.get_logger().info(f'  {i+1}. {action} {args}')

        # add problem to PlanSys2 for record keeping
        self.add_problem_cli.wait_for_service(timeout_sec=10.0)
        req = AddProblem.Request()
        req.problem = problem
        future = self.add_problem_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Problem added to PlanSys2')

        # dispatch first action
        self.current_step = 0
        self._dispatch_next_action()


def main():
    rclpy.init()
    node = ProblemGenerator()
    node.get_logger().info('Waiting for PlanSys2...')
    time.sleep(3.0)
    node.setup_and_trigger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()