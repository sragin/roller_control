# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
import json
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile
from roller_interfaces.action import MoveToPosition
from statemachine import StateMachine, State
from statemachine.exceptions import TransitionNotAllowed
from std_msgs.msg import String

from .path_generator import PathGenerator


class VibrationRollerStateMachine(StateMachine):
    idle = State(initial=True)
    preparing_goal = State()
    navigating = State()

    plan_path = (
        idle.to(preparing_goal)
        | preparing_goal.to(preparing_goal)
    )
    go = preparing_goal.to(navigating)
    stop = (
        idle.to(idle)
        | navigating.to(idle)
    )
    navigation_done = navigating.to(preparing_goal)
    task_done = preparing_goal.to(idle)

    def __init__(self, nav):
        self.navigator :Navigator = nav
        super(VibrationRollerStateMachine, self).__init__()

    def on_enter_idle(self):
        self.navigator.get_logger().warn('idle state')

    def on_enter_preparing_goal(self):
        self.navigator.get_logger().warn('preparing_goal state')
        if self.navigator.plan_path():
            self.navigator.get_logger().warn('Path planning has been done')
            if self.navigator.auto_repeat:
                self.go()
        else:
            self.navigator.get_logger().warn('No more path left')
            if self.navigator.auto_repeat:
                self.navigator.load_pathfile()
                self.plan_path()
            else:
                self.task_done()

    def on_enter_navigating(self):
        self.navigator.get_logger().warn('navigating state')
        self.navigator.go()

    def on_plan_path(self):
        self.navigator.get_logger().warn('Path planning started')

    # def on_go(self):
    #     self.navigator.get_logger().warn('Navigating has been started')

    def on_stop(self):
        self.navigator.get_logger().warn('Stopping machine')
        self.navigator.stop()


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )

        self.sm = VibrationRollerStateMachine(self)
        self._action_client = ActionClient(self, MoveToPosition, 'move_to')

        self.basepoint = [371262.716, 159079.566]
        self.is_first_planning = True
        self.g = self.get_point_from_json()
        self.auto_repeat = False
        self.path_json = None
        self.filename = None

    def recieve_motioncmd(self, msg: String):
        self.get_logger().info(f'{msg}')
        try:
            if msg.data == 'STOP' or msg.data =='E-STOP':
                self.sm.stop()
            elif 'PATHFILE' in msg.data:
                self.load_pathfile(msg.data)
            elif msg.data == 'PLAN PATH':
                self.sm.plan_path()
            elif msg.data == 'START MOTION':
                self.auto_repeat = False
                self.sm.go()
            elif msg.data == 'START TASK':
                self.auto_repeat = True
                self.load_pathfile()
                self.sm.plan_path()
        except TransitionNotAllowed as e:
            self.get_logger().warn(f'{e}')

    def load_pathfile(self, filenamecmd=''):
        if filenamecmd != '':
            _, self.filename = filenamecmd.split(':')
        if self.filename is None:
            return
        with open(self.filename, 'r') as pathfile:
            self.path_json = json.load(pathfile)
        self.get_logger().info(f'Path file \'{self.filename.split("/")[-1]}\' has been loaded')
        self.get_logger().info(f'{self.path_json}')
        self.is_first_planning = True
        self.g = self.get_point_from_json()

    def get_point_from_json(self):
        yield self.path_json['startPoint']
        if 'wayPoint' in self.path_json:
            for j in self.path_json['wayPoint']:
                yield self.path_json['wayPoint'][j]
        yield self.path_json['endPoint']

    def plan_path(self):
        if self.path_json is None:
            self.get_logger().warn('Select path file first')
            return False
        try:
            if self.is_first_planning:
                self.start_point = next(self.g)
                self.end_point = next(self.g)
                self.is_first_planning = False
            else:
                self.start_point = self.end_point
                self.end_point = next(self.g)
        except StopIteration as e:
            # print(f'{e} stop iteration')
            return False
        print(f'{self.start_point["coordinate"]} {self.end_point["coordinate"]}')
        ref_v = self.start_point['velocity']
        is_backward = ref_v < 0
        s_x = self.start_point['coordinate'][1] - self.basepoint[1]
        s_y = self.start_point['coordinate'][0] - self.basepoint[0]
        s_yaw = self.start_point['heading']
        g_x = self.end_point['coordinate'][1] - self.basepoint[1]
        g_y = self.end_point['coordinate'][0] - self.basepoint[0]
        g_yaw = self.end_point['heading']
        p = PathGenerator(s_x=s_x, s_y=s_y, s_yaw=s_yaw,
                        g_x=g_x, g_y=g_y, g_yaw=g_yaw,
                        ref_v=ref_v, is_backward=is_backward)
        self.map_xs, self.map_ys, self.map_yaws, self.cmd_vel = p.plan_path()

        for i in range(len(self.map_xs)):
            print(f'x:{self.map_xs[i] :.3f},'
                    f' y:{self.map_ys[i] :.3f},'
                    f' theta(deg):{self.map_yaws[i]/np.pi*180 :.3f},'
                    f' vel:{self.cmd_vel[i] :.2f}')
        self.get_logger().debug(f'{ref_v} {is_backward} {s_x :.3f} {s_y :.3f} {g_x :.3f} {g_y :.3f}')
        self.get_logger().info('Path has been loaded.')
        return True

    def go(self):
        self.send_goal()
        self.get_logger().info('Motion has been started')

    def stop(self):
        self.get_logger().info('Motion stop requested')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)
        return

    def send_goal(self):
        self.get_logger().info('send goal')
        goal_msg = MoveToPosition.Goal()
        goal_msg.path_pose = []
        for i in range(len(self.map_xs)):
            pose = Pose2D()
            pose.x = self.map_xs[i]
            pose.y = self.map_ys[i]
            pose.theta = self.map_yaws[i]
            goal_msg.path_pose.append(pose)
        goal_msg.path_cmd_vel = []
        for v in self.cmd_vel:
            twist = Twist()
            twist.linear.x = v
            goal_msg.path_cmd_vel.append(twist)

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: MoveToPosition.Result = future.result().result
        if result.result:
            self.get_logger().info(f'Motion succeeded')
            try:
                self.sm.navigation_done()
            except TransitionNotAllowed as e:
                self.get_logger().warn(f'{e}')
        else:
            self.get_logger().info(f'Motion failed')

    def cancel_done(self, future: Future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Motion stopped')
        else:
            self.get_logger().info('Motion failed to cancel')


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
