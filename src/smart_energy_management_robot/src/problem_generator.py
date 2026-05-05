#!/usr/bin/env python3

import random
import yaml
import os
import rclpy

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.srv import AddProblem
from plansys2_msgs.action import ExecutePlan
from rclpy.action import ActionClient


class ProblemGenerator(Node):

    def __init__(self):
        super().__init__('problem_generator')
        self.waypoints = self._load_waypoint_names()
        self.add_problem_cli = self.create_client(
            AddProblem,
            'problem_expert/add_problem'
        )
        self.execute_plan_cli = ActionClient(
            self,
            ExecutePlan,
            'executor/execute_plan'
        )

    def _load_waypoint_names(self):
        pkg = get_package_share_directory('smart_energy_management_robot')
        path = os.path.join(pkg, 'config', 'waypoints.yaml')
        with open(path) as f:
            data = yaml.safe_load(f)
        return list(data['waypoints'].keys())

    def setup_and_trigger(self):
        # randomly assign start, critical, high waypoints
        shuffled = random.sample(self.waypoints, len(self.waypoints))
        start_wp    = shuffled[0]
        critical_wp = shuffled[1]
        high_wp     = shuffled[2]

        self.get_logger().info(f'Start waypoint:    {start_wp}')
        self.get_logger().info(f'Critical waypoint: {critical_wp}')
        self.get_logger().info(f'High waypoint:     {high_wp}')

        # build objects string
        objects = '  ' + ' '.join(self.waypoints) + ' - waypoint'

        # build init predicates
        init_lines = [f'  (robot_at {start_wp})']
        init_lines.append('  (critical_energy_active)')
        init_lines.append('  (high_energy_active)')
        init_lines.append(f'  (is_critical_wp {critical_wp})')
        init_lines.append(f'  (is_high_wp {high_wp})')
        for wp1 in self.waypoints:
            for wp2 in self.waypoints:
                if wp1 != wp2:
                    init_lines.append(f'  (connected {wp1} {wp2})')

        # build goal string
        goal_lines = ['  (and']
        for wp in self.waypoints:
            goal_lines.append(f'    (visited {wp})')
        goal_lines.append('    (not (critical_energy_active))')
        goal_lines.append('    (not (high_energy_active))')
        goal_lines.append('  )')

        # assemble full PDDL problem string
        problem = (
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

        self.get_logger().info(f'Problem:\n{problem}')

        # send problem to PlanSys2
        self.add_problem_cli.wait_for_service(timeout_sec=10.0)
        req = AddProblem.Request()
        req.problem = problem
        future = self.add_problem_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Problem added successfully')
            self._trigger_plan()
        else:
            self.get_logger().error(
                f'Failed to add problem: {future.result().error_info}'
            )

    def _trigger_plan(self):
        self.get_logger().info('Triggering plan execution...')
        self.execute_plan_cli.wait_for_server(timeout_sec=10.0)
        goal = ExecutePlan.Goal()
        future = self.execute_plan_cli.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Plan execution rejected')
            return
        self.get_logger().info('Plan execution accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback):
        self.get_logger().info(
            f'Executing action: {feedback.feedback.action_execution_status}'
        )

    def _result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Plan completed successfully')
        else:
            self.get_logger().error(f'Plan failed: {result.error_info}')


def main():
    rclpy.init()
    node = ProblemGenerator()
    node.get_logger().info('Waiting for PlanSys2...')
    # give PlanSys2 time to start up
    import time
    time.sleep(3.0)
    node.setup_and_trigger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
