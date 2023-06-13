import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

import cantools
from can_msgs.msg import Frame
from msg_gps_interface.msg import GPSMsg


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
        self.can_msg_commandsv = self.candb_autobox_to_supervisor.get_message_by_name('Command_SV')

        self.callback_group = ReentrantCallbackGroup()
        self.can_msg_subscriber = self.create_subscription(
            Frame,
            'from_can_bus',
            self.recv_autobox_state,
            qos_profile,
            callback_group=self.callback_group
        )
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            qos_profile,
            callback_group=self.callback_group
        )

        self.orientation = [0., 0., 0.]   # orientation
        self.position = [0, 0] # position
        self.response = [0, 0, 0.]

        self.count = 0
        self.log_display_cnt = 50

        self.drum_position_publisher = self.create_publisher(Int32MultiArray, 'drum_position', qos_profile)
        self.drum_orientation_publisher = self.create_publisher(Float32MultiArray, 'drum_orientation', qos_profile)
        self.timer_roller_geometry_msg = self.create_timer(1/50, self.publish_roller_geometry_msg)

        self.canbus_publisher = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        # self.timer_roller_geometry_can = self.create_timer(1/50, self.publish_roller_geometry_can)
        self.timer_commandsv = self.create_timer(1/10, self.publish_commandsv)

    def recv_autobox_state(self, msg):
        if msg.id == self.can_msg_response.frame_id:
            _cur = self.can_msg_response.decode(msg.data)
            self.response[0] = _cur['MODE']
            self.response[1] = _cur['STATUS']
            self.response[2] = _cur['STEER_ANGLE']
            # self.get_logger().info(f"MODE: {_cur['MODE']}, STATUS:{_cur['STATUS']}, STEER_ANGLE:{_cur['STEER_ANGLE']}")
        # elif msg.id == self.can_msg_drum_pos.frame_id:
        #     _cur = self.can_msg_drum_pos.decode(msg.data)
        #     self.position[0] = _cur['DRUM_POS_X']
        #     self.position[1] = _cur['DRUM_POS_Y']
        #     # self.get_logger().info(f"DRUM POS_X: {_cur['DRUM_POS_X']}, POS_Y:{_cur['DRUM_POS_Y']}")
        # elif msg.id == self.can_msg_drum_ori.frame_id:
        #     _cur = self.can_msg_drum_ori.decode(msg.data)
        #     self.orientation[0] = _cur['DRUM_PITCH']
        #     self.orientation[1] = _cur['DRUM_ROLL']
        #     self.orientation[2] = _cur['DRUM_HEAD']
        #     # self.get_logger().info(f"DRUM PITCH: {_cur['DRUM_PITCH']}, ROLL:{_cur['DRUM_ROLL']}, HEAD:{_cur['DRUM_HEAD']}")

    def recv_gpsmsg(self, msg: GPSMsg):
        # self.get_logger().info(f'{msg}')
        self.position[0] = int(msg.tm_x * 1000)
        self.position[1] = int(msg.tm_y * 1000)
        self.orientation[2] = msg.heading - 90

    def publish_roller_geometry_msg(self):
        msg_pos = Int32MultiArray()
        msg_pos.data = self.position
        self.drum_position_publisher.publish(msg_pos)

        msg_ori = Float32MultiArray()
        msg_ori.data = self.orientation
        self.drum_orientation_publisher.publish(msg_ori)
        if self.count == self.log_display_cnt:
            self.get_logger().info(f"MODE: {self.response[0]}, STATUS:{self.response[1]}, STEER_ANGLE:{self.response[2] :.1f}")
            self.get_logger().info(f"DRUM POS_X: {self.position[0]}, POS_Y:{self.position[1]}")
            self.get_logger().info(f"DRUM PITCH: {self.orientation[0] :.1f}, ROLL:{self.orientation[1] :.1f}, HEAD:{self.orientation[2] :.1f}")
            self.count = 0
        self.count += 1

    def publish_roller_geometry_can(self):
        data = self.can_msg_drum_pos.encode(
            {'DRUM_POS_X':self.position[0],
             'DRUM_POS_Y':self.position[1]}
        )
        send_msg = Frame()
        send_msg.id = self.can_msg_drum_pos.frame_id
        send_msg.dlc = self.can_msg_drum_pos.length
        for i in range(send_msg.dlc):
            send_msg.data[i] = int(data[i])
        self.canbus_publisher.publish(send_msg)

        data = self.can_msg_drum_ori.encode(
            {'DRUM_PITCH':self.orientation[0],
             'DRUM_ROLL':self.orientation[1],
             'DRUM_HEAD':self.orientation[2]}
        )
        send_msg = Frame()
        send_msg.id = self.can_msg_drum_ori.frame_id
        send_msg.dlc = self.can_msg_drum_ori.length
        for i in range(send_msg.dlc):
            send_msg.data[i] = int(data[i])
        self.canbus_publisher.publish(send_msg)

    def publish_commandsv(self):
        data = self.can_msg_commandsv.encode(
            {'MODE':1,
             'AUTO_DRIVE':0,
             'STOP_CMD':0}
        )
        send_msg = Frame()
        send_msg.id = self.can_msg_commandsv.frame_id
        send_msg.dlc = self.can_msg_commandsv.length
        for i in range(send_msg.dlc):
            send_msg.data[i] = int(data[i])
        self.canbus_publisher.publish(send_msg)


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
