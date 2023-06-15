import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus

from .path_generator import PathGenerator


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

        self.path_generator = PathGenerator()

        self.orientation = [0., 0., 0.]   # orientation
        self.position = [0, 0] # position

        self.count = 0
        self.log_display_cnt = 50

    def control(self):
        pass

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.get_logger().info(f'{msg}')


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
