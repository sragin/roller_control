import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose
from roller_interfaces.msg import RollerStatus


class TurtleSimInterface(Node):
    def __init__(self):
        super().__init__('roller_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        qos_profile = QoSProfile(depth=10)
        self.turtlepose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.recieve_pose,
            qos_profile
        )
        self.rollerpose_publisher = self.create_publisher(RollerStatus, 'roller_status', qos_profile)
    
    def recieve_pose(self, msg: Pose):
        r_msg = RollerStatus()
        r_msg.header.frame_id = 'world'
        r_msg.header.stamp = self.get_clock().now().to_msg()
        r_msg.steer_angle.data = 0.0
        r_msg.pose.theta = msg.theta
        r_msg.pose.x = msg.x
        r_msg.pose.y = msg.y
        self.rollerpose_publisher.publish(r_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        roller_controller = TurtleSimInterface()
        rclpy.spin(roller_controller)
    except KeyboardInterrupt:
        roller_controller.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        roller_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()