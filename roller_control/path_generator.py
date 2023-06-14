import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray


class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        qos_profile = QoSProfile(depth=10)
        self.rollerpose_subscriber = self.create_subscription(
            Int32MultiArray,
            'drum_position',
            self.recieve_rollerpose_msg,
            qos_profile
        )

        self.orientation = [0., 0., 0.]   # orientation
        self.position = [0, 0] # position

        self.count = 0
        self.log_display_cnt = 50

    def recieve_rollerpose_msg(self, msg: Int32MultiArray):
        self.position[0] = msg.data[0]
        self.position[1] = msg.data[1]
        print(self.position)


def main(args=None):
    rclpy.init(args=args)
    try:
        path_generator = PathGenerator()
        rclpy.spin(path_generator)
    except KeyboardInterrupt:
        path_generator.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        path_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
