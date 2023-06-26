from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String

from .control_algorithm import MAX_STEER_LIMIT, MAX_STEER_VEL
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

        self.map_xs = None
        self.map_ys = None
        self.map_yaws = None
        self.cmd_vel = None

        self.roller_status = RollerStatus()

        self.count = 0
        self.log_display_cnt = 50

    def control(self):
        dt = CONTROL_PERIOD
        x = self.roller_status.pose.x
        y = self.roller_status.pose.y
        theta = self.roller_status.pose.theta
        steer_angle = self.roller_status.steer_angle
        vel = self.cmd_vel[0]

        # 종료조건 계산
        error = 0.05
        if self.map_xs[-1] > self.map_xs[0]:
            if self.map_xs[-1] - error <= x:
                self.control_timer.cancel()
                self.control_timer = None
        elif self.map_xs[-1] < self.map_xs[0]:
            if self.map_xs[-1] - error >= x:
                self.control_timer.cancel()
                self.control_timer = None
        if self.map_ys[-1] > self.map_ys[0]:
            if self.map_ys[-1] - error <= y:
                self.control_timer.cancel()
                self.control_timer = None
        elif self.map_ys[-1] < self.map_ys[0]:
            if self.map_ys[-1] - error >= y:
                self.control_timer.cancel()
                self.control_timer = None
        if self.control_timer is None:
            self.get_logger().info('motion done')
            return

        steer_, yaw_, cte_, min_dist_, min_index_ =\
            stanley_control(x, y, vel, theta + steer_angle, self.map_xs, self.map_ys, self.map_yaws)
        steer_cmd = np.clip(steer_, -MAX_STEER_LIMIT, MAX_STEER_LIMIT)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.cmd_vel[min_index_]
        cmd_vel_msg.angular.z = steer_cmd
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(
            f'steer_:{steer_ :.3f}, yaw:{yaw_ :.3f}, cte:{cte_ :.3f},'
            f' min_dist:{min_dist_ :.3f} idx:{min_index_}\n'
            f'xs:{self.map_xs[0] :.3f} xe:{self.map_xs[-1] :.3f} x:{x :.3f} '
            f'ys:{self.map_ys[0] :.3f} ye:{self.map_ys[-1] :.3f} y:{y :.3f}\n'
            f'steer(rad):{steer_angle :.3f} steer_cmd(rad):{steer_cmd :.3f}'
            f' yaws:{self.map_yaws[0] :.3f} yaw:{theta + steer_angle :.3f} cmd_vel:{self.cmd_vel[min_index_]}')

    def recieve_motioncmd(self, msg):
        # self.get_logger().info(f'{msg}')
        if msg.data == 'STOP':
            if self.control_timer is not None:
                self.control_timer.cancel()
                self.control_timer = None
                self.get_logger().info('control algorithm has been stopped')
                return
        elif msg.data == 'PATH':
            if self.control_timer is None:
                p = PathGenerator()
                self.map_xs, self.map_ys, self.map_yaws, self.cmd_vel = p.generate_path()
                self.get_logger().info('path stamped has been loaded')
            else:
                self.get_logger().info('control algorithm is already running')
                return
        elif msg.data == 'START':
            if self.map_xs is None:
                self.get_logger().warn('path is empty')
                return

            if self.control_timer is None:
                self.control_timer = self.create_timer(CONTROL_PERIOD, self.control)
            else:
                self.get_logger().info('control algorithm is already running')
                return

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
