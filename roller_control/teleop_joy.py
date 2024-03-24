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
            qos_profile=10
        )
        self.teleop_msg_publisher = self.create_publisher(RollerTeleop, 'teleop_cmd', qos_profile=10)

        self.count = 0
        self.log_display_cnt = 50

    # 아날로그 모듈에서 입력된 조이스틱 값을 양자화, 노말라이즈 0.5 ~ 4.5V 범위
    # 아날로그 모듈은 0 ~ 10V 입력 가능하도록 세팅됨
    # 장비에 의존적임. 각각의 장비에서 사용하는 범위의 값으로 변환해서 송신해줌
    def recv_joystic_message(self, msg: String):
        AD_MID_VAL = 16383
        AD_MAX_VAL = 29490
        AD_MIN_VAL = 3276
        AD_RANGE = AD_MAX_VAL - AD_MIN_VAL

        data = json.loads(msg.data)
        _drive = data['ch5']
        _steer = data['ch2']
        drive = (_drive - AD_MID_VAL) / AD_RANGE
        steer = (_steer - AD_MID_VAL) / AD_RANGE

        teleop_msg = RollerTeleop()
        teleop_msg.drive = int(drive)
        if steer > 0:
            teleop_msg.steer_left = int(steer)
        else:
            teleop_msg.steer_right = int(abs(steer))
        self.teleop_msg_publisher.publish(teleop_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().info(f"Received: {msg.data}")
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
