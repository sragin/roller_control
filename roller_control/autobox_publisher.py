import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cantools
from can_msgs.msg import Frame


class AutoboxPublisher(Node):
    def __init__(self):
        super().__init__('autobox_publisher')
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        qos_profile = QoSProfile(depth=10)
        self.candb_autobox_to_supervisor = cantools.db.load_file('./install/roller_control/share/ToSupervisor_210430.dbc')
        self.can_msg_response = self.candb_autobox_to_supervisor.get_message_by_name('Response')

        self.callback_group = ReentrantCallbackGroup()
        self.can_msg_subscriber = self.create_subscription(
            Frame,
            'from_can_bus',
            self.recv_autobox_state,
            qos_profile,
            callback_group=self.callback_group
        )
        self.count = 0
        self.log_display_cnt = 10

    def recv_autobox_state(self, msg):
        if msg.id == self.can_msg_response.frame_id:
            _cur = self.can_msg_response.decode(msg.data)
            if self.count == self.log_display_cnt:
                self.get_logger().info(f"MODE: {_cur['MODE']}, STATUS:{_cur['STATUS']}, STEER_ANGLE:{_cur['STEER_ANGLE']}")
                self.count = 0
            self.count += 1

        if self.count == self.log_display_cnt:
            # self.get_logger().info(f'Received: {msg}')
            # self.get_logger().warning(f'Controller Command id: {send_msg.id} data: {send_msg.data}')
            # self.get_logger().warning(f'Supervisor Command id: {send_msg2.id} data: {send_msg2.data}')
            self.count = 0
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        autobox_publisher = AutoboxPublisher()
        rclpy.spin(autobox_publisher)
    except KeyboardInterrupt:
        autobox_publisher.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        autobox_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
