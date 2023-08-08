# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from geometry_msgs.msg import Twist
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String

from .control_algorithm import MAX_STEER_LIMIT
from .control_algorithm import stanley_control
from .path_generator import PathGenerator

CONTROL_PERIOD = 0.1


class RollerController(Node):

    def __init__(self):
        super().__init__('roller_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollerstatus_subscriber = self.create_subscription(
            RollerStatus,
            'roller_status',
            self.recieve_rollerstatus,
            qos_profile)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile)
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

        if self.check_goal(self.map_xs, self.map_ys, x, y, self.goal_check_error):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.control_timer.cancel()
            self.control_timer = None
            self.get_logger().info('motion done')

        self.cmd_vel_publisher.publish(cmd_vel_msg)
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

    def recieve_motioncmd(self, msg: String):
        self.get_logger().info(f'{msg}')
        if msg.data == 'STOP':
            if self.control_timer is not None:
                self.control_timer.cancel()
                self.control_timer = None
                self.get_logger().info('control algorithm has been stopped')
                return
        elif 'PATHFILE' in msg.data:
            _, filename = msg.data.split(':')
            with open(filename, 'r') as pathfile:
                self.path_json = json.load(pathfile)
            self.get_logger().info(f'Path file \'{filename.split("/")[-1]}\' has been loaded')
            self.get_logger().info(f'{self.path_json}')
        elif msg.data == 'PLAN PATH':
            ref_v = self.path_json['startPoint']['velocity']
            is_backward = ref_v < 0
            s_x = self.path_json['startPoint']['coordinate'][0] - self.basepoint[1]
            s_y = self.path_json['startPoint']['coordinate'][1] - self.basepoint[0]
            s_yaw = self.path_json['startPoint']['heading']
            g_x = self.path_json['endPoint']['coordinate'][0] - self.basepoint[1]
            g_y = self.path_json['endPoint']['coordinate'][1] - self.basepoint[0]
            g_yaw = self.path_json['endPoint']['heading']
            p = PathGenerator(s_x=s_x, s_y=s_y, s_yaw=s_yaw,
                            g_x=g_x, g_y=g_y, g_yaw=g_yaw,
                            ref_v=ref_v, is_backward=is_backward)
            map_xs, map_ys, map_yaws, cmd_vel = p.plan_path()
            self.get_logger().info(f'{ref_v} {is_backward}')

            import numpy as np
            for i in range(len(map_xs)):
                print(f'x:{map_xs[i] :.3f},'
                        f' y:{map_ys[i] :.3f},'
                        f' theta(deg):{map_yaws[i]/np.pi*180 :.3f},'
                        f' vel:{cmd_vel[i] :.2f}')
            self.get_logger().info('path stamped has been loaded')
        elif msg.data == 'START MOTION':
            if self.map_xs is None:
                self.get_logger().warn('path is empty')
                return

            if self.control_timer is None:
                self.control_timer = self.create_timer(CONTROL_PERIOD, self.control)
            else:
                self.get_logger().info('control algorithm is already running')
                return
        elif msg.data == 'START TASK':
            pass

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.roller_status = msg


def main(args=None):
    rclpy.init(args=args)
    try:
        roller_controller = RollerController()
        rclpy.spin(roller_controller)
    except KeyboardInterrupt:
        roller_controller.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        roller_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
