import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from teleop_msgs.msg import RollerTeleop


def clamp(n, minn = -100, maxn = 100):
    return max(min(maxn, n), minn)


class TeleopJoyPublisher(Node):
    def __init__(self):
        super().__init__('teleop_joy')

        self.joystic_msg_subscriber = self.create_subscription(
            String,
            'joystick',
            self.recv_joystic_message,
            qos_profile=5
        )
        self.teleop_msg_publisher = self.create_publisher(RollerTeleop, 'teleop_cmd', qos_profile=5)

        self.count = 0
        self.log_display_cnt = 50

    # 아날로그 모듈은 0 ~ 10V 의 입력을 받을 수 있으나 조이스틱 출력은 0.5 ~ 4.5V 의 범위임
    # 장비에 의존적임. 각각의 장비에서 사용하는 범위의 값으로 변환해서 송신해줌
    def recv_joystic_message(self, msg: String):
        AD_MID_VAL = 2048 # 2.5V
        AD_MAX_VAL = 4096 # 5V
        AD_MIN_VAL = 0
        AD_RANGE = (AD_MAX_VAL - AD_MIN_VAL) / 2

        data = json.loads(msg.data)
        _drive = data['ch5']
        _steer = data['ch2']
        drive = (_drive - AD_MID_VAL) / AD_RANGE * 1000
        steer = (_steer - AD_MID_VAL) / AD_RANGE * 100

        teleop_msg = RollerTeleop()
        if not (-25 < drive < 50):
            teleop_msg.drive = int(drive)
        if steer > 5:
            teleop_msg.steer_right = int(steer)
        elif steer < -2.5:
            teleop_msg.steer_left = int(abs(steer))
        self.teleop_msg_publisher.publish(teleop_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().info(f"Received: {msg.data}")
            self.get_logger().info(f"DRV: {drive}, STEER: {steer}")
            self.get_logger().info(f"Published message: {teleop_msg}")
            self.count = 0
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    exteleopjoy_publisher = TeleopJoyPublisher()
    try:
        exteleopjoy_publisher.get_logger().info(f'teleop_joy started.')
        rclpy.spin(exteleopjoy_publisher)
    except KeyboardInterrupt:
        exteleopjoy_publisher.get_logger().warn('Keyboard interrupt (SIGINT)')
    finally:
        exteleopjoy_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
