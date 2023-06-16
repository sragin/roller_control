import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String

from .path_generator import PathGenerator
import numpy as np
from .control_algorithm import stanley_control


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
            qos_profile
        )
        self.rollerstatus_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )
        self.control_timer = None

        self.map_xs = None
        self.map_ys = None
        self.map_yaws = None
        self.cmd_vel = None
        self.orientation = [0., 0., 0.]   # orientation
        self.position = [0, 0] # position

        self.count = 0
        self.log_display_cnt = 50

    def control(self):
        steer_, yaw_, cte_, min_dist_ = stanley_control(self.position[0], self.position[1], self.cmd_vel[0], self.orientation[2], self.map_xs, self.map_ys, self.map_yaws)
        self.get_logger().info(f"steer(deg):{steer_ * 180 / np.pi :.3f}, yaw:{yaw_ :.3f}, cte:{cte_ :.3f}, min_dist:{min_dist_ :.3f}")
        self.get_logger().info(f'{self.map_xs[0] :.3f} {self.map_xs[-1] :.3f} {self.position[0] :.3f} {self.map_ys[0] :.3f} {self.map_ys[-1] :.3f} {self.position[1] :.3f}')

        # 종료조건 계산
        error = 0.05
        if self.map_xs[-1] > self.map_xs[0]:
            if self.map_xs[-1] - error <= self.position[0]:
                self.control_timer.cancel()
                self.control_timer = None
                return
        elif self.map_xs[-1] < self.map_xs[0]:
            if self.map_xs[-1] - error >= self.position[0]:
                self.control_timer.cancel()
                self.control_timer = None
                return
        if self.map_ys[-1] > self.map_ys[0]:
            if self.map_ys[-1] - error <= self.position[1]:
                self.control_timer.cancel()
                self.control_timer = None
                return
        elif self.map_ys[-1] < self.map_ys[0]:
            if self.map_ys[-1] - error >= self.position[1]:
                self.control_timer.cancel()
                self.control_timer = None
                return

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
                self.control_timer = self.create_timer(0.1, self.control)
            else:
                self.get_logger().info('control algorithm is already running')
                return

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.position[0] = msg.pose.x
        self.position[1] = msg.pose.y
        self.orientation[2] = msg.pose.theta


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
