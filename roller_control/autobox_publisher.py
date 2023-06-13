import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

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
        self.can_msg_drum_pos = self.candb_autobox_to_supervisor.get_message_by_name('Drum_Position')
        self.can_msg_drum_ori = self.candb_autobox_to_supervisor.get_message_by_name('Drum_Orientation')

        self.callback_group = ReentrantCallbackGroup()
        self.can_msg_subscriber = self.create_subscription(
            Frame,
            'from_can_bus',
            self.recv_autobox_state,
            qos_profile,
            callback_group=self.callback_group
        )

        self.orientation = [0., 0., 0.]   # orientation
        self.position = [0, 0] # position

        self.count = 0
        self.log_display_cnt = 50

        self.drum_position_publisher = self.create_publisher(Int32MultiArray, 'drum_position', qos_profile)
        self.drum_orientation_publisher = self.create_publisher(Float32MultiArray, 'drum_orientation', qos_profile)
        self.timer = self.create_timer(1/50, self.publish_roller_geometry_msg)

    def recv_autobox_state(self, msg):
        if msg.id == self.can_msg_response.frame_id:
            _cur = self.can_msg_response.decode(msg.data)
            # self.get_logger().info(f"MODE: {_cur['MODE']}, STATUS:{_cur['STATUS']}, STEER_ANGLE:{_cur['STEER_ANGLE']}")
        elif msg.id == self.can_msg_drum_pos.frame_id:
            _cur = self.can_msg_drum_pos.decode(msg.data)
            self.position[0] = _cur['DRUM_POS_X']
            self.position[1] = _cur['DRUM_POS_Y']
            # self.get_logger().info(f"DRUM POS_X: {_cur['DRUM_POS_X']}, POS_Y:{_cur['DRUM_POS_Y']}")
        elif msg.id == self.can_msg_drum_ori.frame_id:
            _cur = self.can_msg_drum_ori.decode(msg.data)
            self.orientation[0] = _cur['DRUM_PITCH']
            self.orientation[1] = _cur['DRUM_ROLL']
            self.orientation[2] = _cur['DRUM_HEAD']
            # self.get_logger().info(f"DRUM PITCH: {_cur['DRUM_PITCH']}, ROLL:{_cur['DRUM_ROLL']}, HEAD:{_cur['DRUM_HEAD']}")

    def publish_roller_geometry_msg(self):
        msg = Int32MultiArray()
        msg.data = self.position
        self.drum_position_publisher.publish(msg)
        msg = Float32MultiArray()
        msg.data = self.orientation
        self.drum_orientation_publisher.publish(msg)
        if self.count == self.log_display_cnt:
            # self.get_logger().info(f'Received: {msg}')
            # self.get_logger().info(f"MODE: {_cur['MODE']}, STATUS:{_cur['STATUS']}, STEER_ANGLE:{_cur['STEER_ANGLE']}")
            self.get_logger().info(f"DRUM POS_X: {self.position[0]}, POS_Y:{self.position[1]}")
            self.get_logger().info(f"DRUM PITCH: {self.orientation[0] :.1f}, ROLL:{self.orientation[1] :.1f}, HEAD:{self.orientation[2] :.1f}")
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
