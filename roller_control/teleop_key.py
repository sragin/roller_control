import rclpy
from rclpy.node import Node

from teleop_msgs.msg import RollerTeleop
from pynput import keyboard


class RollerTeleopKeyPublisher(Node):
    def __init__(self):
        super().__init__('roller_teleop_key')
        self.publisher_ = self.create_publisher(RollerTeleop, 'roller_teleop_cmd', 10)
        self.timer_ = self.create_timer(0.1, self.publish_commands)
        self.command_ = [0, 0, 0]
        self.MAX_POWER = 20
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
        self.get_logger().warn('Roller_teleop_key node started')

    def on_press(self, key):
        self.get_logger().info(f'Key {key} pressed')
        if key == keyboard.Key.up:
            self.command_[2] = self.MAX_POWER
        elif key == keyboard.Key.down:
            self.command_[2] = -self.MAX_POWER
        if key == keyboard.Key.left:
            self.command_[0] = self.MAX_POWER
            self.command_[1] = 0
        elif key == keyboard.Key.right:
            self.command_[0] = 0
            self.command_[1] = self.MAX_POWER

    def on_release(self, key):
        self.get_logger().info(f'Key {key} released')
        self.command_[0] = 0
        self.command_[1] = 0
        self.command_[2] = 0

    def publish_commands(self):
        msg = RollerTeleop()
        msg.left_duty_control = self.command_[0]
        msg.right_duty_control = self.command_[1]
        msg.auto_spd_control = self.command_[2]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')


def main(args=None):
    rclpy.init(args=args)
    rollerteleopkey_publisher = RollerTeleopKeyPublisher()
    try:
        rclpy.spin(rollerteleopkey_publisher)
    except KeyboardInterrupt:
        msg = RollerTeleop()
        msg.left_duty_control = 0
        msg.right_duty_control = 0
        msg.auto_spd_control = 0
        rollerteleopkey_publisher.publisher_.publish(msg)
        rollerteleopkey_publisher.get_logger().info('Set all commands to zero')
        rollerteleopkey_publisher.get_logger().info('Keyboard interrupt (SIGINT)')
    rollerteleopkey_publisher.listener.join()
    rollerteleopkey_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
