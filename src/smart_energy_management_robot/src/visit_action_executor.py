#!/usr/bin/env python3

import sys
import os
import yaml
import py_trees
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.msg import ActionExecution

sys.path.insert(0, os.path.join(
    get_package_share_directory('smart_energy_management_robot'), 'src'
))
from bt_nodes import create_inspection_tree


def load_waypoints():
    pkg = get_package_share_directory('smart_energy_management_robot')
    path = os.path.join(pkg, 'config', 'waypoints.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return data['waypoints']


class VisitActionExecutor(Node):
    """
    Listens to PlanSys2 action dispatch topic.
    When a visit action is received, navigates to the waypoint
    then runs the inspection BT.
    """

    def __init__(self):
        super().__init__('visit_action_executor')
        self.waypoints = load_waypoints()
        self.nav_done = False
        self.nav_success = False
        self.current_action = None

        # subscribe to PlanSys2 action execution topic
        self.action_sub = self.create_subscription(
            ActionExecution,
            'actions_hub',
            self._action_cb,
            10
        )

        # publish action execution responses back to PlanSys2
        self.action_pub = self.create_publisher(
            ActionExecution,
            'actions_hub',
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('VisitActionExecutor ready')

    def _action_cb(self, msg):
        # only handle REQUEST messages for our actions
        if msg.type != ActionExecution.REQUEST:
            return
        if msg.action not in ['visit_critical', 'visit_high', 'visit_waypoint']:
            return

        self.current_action = msg
        # target waypoint is second argument (from, to)
        target = msg.arguments[1]
        self.get_logger().info(
            f'Received action: {msg.action} -> {target}'
        )

        # send CONFIRM back to PlanSys2
        self._send_response(ActionExecution.CONFIRM, target, 0.0, 'Navigating')
        self.nav_done = False
        self.nav_success = False
        self._navigate(target)

    def _navigate(self, waypoint: str):
        coords = self.waypoints.get(waypoint)
        if coords is None:
            self.get_logger().error(f'Unknown waypoint: {waypoint}')
            self._send_response(
                ActionExecution.FINISH, waypoint, 0.0, 'Unknown waypoint', False
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(coords[0])
        goal.pose.pose.position.y = float(coords[1])
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to {waypoint} {coords}')
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._nav_response_cb(f, waypoint)
        )

    def _nav_response_cb(self, future, waypoint):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._send_response(
                ActionExecution.FINISH, waypoint, 0.0, 'Nav rejected', False
            )
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._nav_result_cb(f, waypoint)
        )

    def _nav_result_cb(self, future, waypoint):
        self.get_logger().info(f'Arrived at {waypoint}')
        self._send_response(
            ActionExecution.FEEDBACK, waypoint, 0.5, 'Arrived, inspecting'
        )
        self._run_inspection(waypoint)

    def _run_inspection(self, waypoint: str):
        tree = create_inspection_tree(waypoint)
        tree.setup_with_descendants()
        tree.tick_once()

        if tree.status == py_trees.common.Status.SUCCESS:
            self.get_logger().info(f'Inspection complete: {waypoint}')
            self._send_response(
                ActionExecution.FINISH, waypoint, 1.0, 'Done', True
            )
        else:
            self.get_logger().error(f'Inspection failed: {waypoint}')
            self._send_response(
                ActionExecution.FINISH, waypoint, 0.0, 'Inspection failed', False
            )

    def _send_response(self, msg_type, waypoint, completion, status, success=True):
        if self.current_action is None:
            return
        msg = ActionExecution()
        msg.type = msg_type
        msg.node_id = self.get_name()
        msg.action = self.current_action.action
        msg.arguments = self.current_action.arguments
        msg.success = success
        msg.completion = completion
        msg.status = status
        self.action_pub.publish(msg)


def main():
    rclpy.init()
    node = VisitActionExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
