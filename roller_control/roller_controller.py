# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.action import MoveToPosition
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String
import time

from .control_algorithm import MAX_STEER_LIMIT
from .control_algorithm import stanley_control

CONTROL_PERIOD = 0.1


class RollerController(Node):

    def __init__(self):
        super().__init__('roller_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )
        self.rollerstatus_subscriber = self.create_subscription(
            RollerStatus,
            'roller_status',
            self.recieve_rollerstatus,
            qos_profile)
        self._action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.control_timer = None

        self.basepoint = [371262.716, 159079.566]
        self.path_json = None
        self.map_xs = None
        self.map_ys = None
        self.map_yaws = None
        self.cmd_vel = None
        self.goal_check_error = 0.05

        self.roller_status = RollerStatus()

        self.count = 0
        self.log_display_cnt = 50

    def recieve_motioncmd(self, msg: String):
        if msg.data =='E-STOP':
            self.get_logger().info(f'{msg}')
            if self.control_timer is not None:
                self.control_timer.cancel()
                self.control_timer = None

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal')
        feedback_msg = MoveToPosition.Feedback()
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control)

        while self.control_timer is not None:
            if goal_handle.is_cancel_requested:
                self.control_timer.cancel()
                self.control_timer = None
            time.sleep(0.1)

        result = MoveToPosition.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.result = False
            self.get_logger().info('Goal has been canceled')
        else:
            goal_handle.succeed()
            result.result = True
        return result

    def goal_callback(self, goal: MoveToPosition.Goal):
        self.map_xs = [p.x for p in goal.path_pose]
        self.map_ys = [p.y for p in goal.path_pose]
        self.map_yaws = [p.theta for p in goal.path_pose]
        self.cmd_vel = [v.linear.x for v in goal.path_cmd_vel]
        self.get_logger().info(f'{self.map_xs} {self.map_ys} {self.map_yaws} {self.cmd_vel}')
        if len(self.map_xs) == len(self.map_ys) == len(self.map_yaws) == len(self.cmd_vel):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def control(self):
        """
        제어 알고리즘 실행구문.
        * 전진시 롤러 드럼을 기준으로 제어
        * 후진시 바디를 기준으로 제어
        -X방향(후진주행방향)을 +X방향으로 변환
        왼손좌표계사용 (좌회전:음수, 우회전:양수)
        """
        vel = self.cmd_vel[0]
        theta = self.roller_status.pose.theta

        if vel < 0:
            x = self.roller_status.body_pose.x
            y = self.roller_status.body_pose.y
            theta_ = theta + np.pi
            steer_angle = self.roller_status.steer_angle
        else:
            x = self.roller_status.pose.x
            y = self.roller_status.pose.y
            steer_angle = self.roller_status.steer_angle
            theta_ = theta + steer_angle

        steer_, yaw_, cte_, min_dist_, min_index_ =\
            stanley_control(x=x, y=y, yaw=theta_, v=vel,
                            map_xs=self.map_xs, map_ys=self.map_ys, map_yaws=self.map_yaws)
        steer_cmd = np.clip(steer_, -MAX_STEER_LIMIT, MAX_STEER_LIMIT)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.cmd_vel[min_index_]
        cmd_vel_msg.angular.z = steer_cmd

        self.get_logger().info(
            f'Controller = steer_:{steer_ :.3f}, yaw_:{yaw_ :.3f}, cte_:{cte_ :.3f}, '
            f'min_dist_:{min_dist_ :.3f} idx:{min_index_}\n'
            f'MAP = xs:{self.map_xs[0] :.3f} xe:{self.map_xs[-1] :.3f} '
            f'xi:{self.map_xs[min_index_] :.3f} x:{x :.3f} '
            f'ys:{self.map_ys[0] :.3f} ye:{self.map_ys[-1] :.3f} '
            f'yi:{self.map_ys[min_index_] :.3f} y:{y :.3f} '
            f'yaw goal:{self.map_yaws[min_index_] * 180 / np.pi :.3f} '
            f'yaw:{(theta + steer_angle) * 180 / np.pi :.3f}\n'
            f'Roller Status = steer angle:{steer_angle * 180 / np.pi :.3f} '
            f'steer_cmd:{steer_cmd * 180 / np.pi :.3f} '
            f'heading:{theta * 180 / np.pi :.3f} '
            f'cmd_vel:{self.cmd_vel[min_index_]}')

        if self.check_goal(self.map_xs, self.map_ys, x, y, self.goal_check_error):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.control_timer.cancel()
            self.control_timer = None
            self.get_logger().info('Motion is done')
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def check_goal(self, map_xs, map_ys, x, y, error):
        x1 = map_xs[-1]
        y1 = map_ys[-1]

        # 거리가 error 보다 작으면 무조건 종료
        dx = x1 - x
        dy = y1 - y
        dist = np.sqrt(dx*dx + dy*dy)
        if dist < error:
            # print('Distance to GOAL is less than error')
            return True

        """
        시작점(x)을 종료점(x1) 기준으로 90도 회전한 점을 x2라고 정의
        x1에서 x2 방향으로 이어지는 직선 왼쪽영역을 종료조건으로 하여
        현재위치가 왼쪽에 있는지 오른쪽에 있는지 벡터의 외적을 이용해서 계산한다
        """
        x2_trans1 = map_xs[0] - x1
        y2_trans1 = map_ys[0] - y1
        theta = np.pi/2
        x2_rot = x2_trans1 * np.cos(theta) - y2_trans1 * np.sin(theta)
        y2_rot = x2_trans1 * np.sin(theta) + y2_trans1 * np.cos(theta)
        x2 = x2_rot + x1
        y2 = y2_rot + y1

        cross = (x2 - x1)*(y - y1) - (x - x1) * (y2 - y1)
        # print(f'[{x1:.2f}, {y1:.2f}] [{x2:.2f}, {y2:.2f}] [{x:.2f}, {y:.2f}] {cross:.2f}')
        return cross >= 0

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.roller_status = msg


def main(args=None):
    rclpy.init(args=args)

    roller_controller = RollerController()
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(roller_controller, executor=executor)
    roller_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
