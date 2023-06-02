import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cantools
from teleop_msgs.msg import RollerTeleop
from can_msgs.msg import Frame


class RollerTeleop2CanPublisher(Node):
    def __init__(self):
        super().__init__('roller_teleop2can')
        qos_profile = QoSProfile(depth=10)
        self.candb_controller = cantools.db.load_file('./install/roller_control/share/Controller_230518.dbc')
        self.can_msg_control = self.candb_controller.get_message_by_name('CONTROLLER_COMM')

        self.callback_group = ReentrantCallbackGroup()
        self.can_msg_subscriber = self.create_subscription(
            RollerTeleop,
            'roller_teleop_cmd',
            self.recv_teleop_cmd,
            qos_profile,
            callback_group=self.callback_group
        )
        self.publisher_ = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.count = 0
        self.log_display_cnt = 10

    # 키보드나 조이스틱 조작으로 생성된 토픽(ex25_teleop_cmd/Exteleop 메시지)을
    # HYDAC 송신용 캔 패킷으로 변환 후 socketcan용 토픽(to_can_bus/Frame 메시지)으로 던진다
    def recv_teleop_cmd(self, msg):
        data = self.can_msg_control.encode(
            {'LEFT_DUTY_CONTROL':msg.left_duty_control,
             'RIGHT_DUTY_CONTROL':msg.right_duty_control,
             'AUTO_SPD_CONTROL':msg.auto_spd_control,
             'UNUSED_CONTROL':0}
            )
        send_msg = Frame()
        send_msg.id = self.can_msg_control.frame_id
        for i in range(8):
            send_msg.data[i] = int(data[i])

        send_msg.dlc = self.can_msg_control.length
        self.publisher_.publish(send_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().info(f'Received: {msg}')
            self.get_logger().warning(f'Controller Command id: {send_msg.id} data: {send_msg.data}')
            self.count = 0
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        rollerteleop2can_publisher = RollerTeleop2CanPublisher()
        rclpy.spin(rollerteleop2can_publisher)
    except KeyboardInterrupt:
        rollerteleop2can_publisher.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        rollerteleop2can_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
