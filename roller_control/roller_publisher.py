import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus

import cantools
from can_msgs.msg import Frame
from msg_gps_interface.msg import GPSMsg


class RollerPublisher(Node):
    def __init__(self):
        super().__init__('roller_publisher')
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        qos_profile = QoSProfile(depth=10)
        self.candb_autobox_to_supervisor = cantools.db.load_file('./install/roller_control/share/ToSupervisor_230619.dbc')
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
        self.canbus_publisher = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.roller_status_publisher = self.create_publisher(RollerStatus, 'roller_status', qos_profile)

        self.timer_commandsv = self.create_timer(1/10, self.publish_commandsv)
        self.timer_roller_geometry_msg = self.create_timer(1/50, self.publish_roller_geometry_msg)

        self.theta = 0.0
        self.steer_angle = 0.0
        self.position = [0., 0.] # position
        self.response = [0, 0]

        self.count = 0
        self.log_display_cnt = 50


    def recv_autobox_state(self, msg):
        if msg.id == self.can_msg_response.frame_id:
            _cur = self.can_msg_response.decode(msg.data)
            self.response[0] = _cur['MODE']
            self.response[1] = _cur['STATUS']
            self.steer_angle = _cur['STEER_ANGLE']

    def recv_gpsmsg(self, msg: GPSMsg):
        self.position[0] = msg.tm_x - 371400
        self.position[1] = msg.tm_y - 159200
        self.theta = msg.heading - 90

    def publish_roller_geometry_msg(self):
        msg = RollerStatus()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steer_angle.data = self.steer_angle
        msg.pose.theta = self.theta
        msg.pose.x = self.position[0]
        msg.pose.y = self.position[1]
        self.roller_status_publisher.publish(msg)
        if self.count == self.log_display_cnt:
            self.get_logger().info(f"MODE: {self.response[0]}, STATUS:{self.response[1]}, STEER_ANGLE:{self.steer_angle :.2f}")
            self.get_logger().info(f"DRUM POS_X: {self.position[0] :.4f}, POS_Y:{self.position[1] :.4f}, HEAD:{self.theta :.1f}")
            self.count = 0
        self.count += 1

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
        roller_publisher = RollerPublisher()
        rclpy.spin(roller_publisher)
    except KeyboardInterrupt:
        roller_publisher.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        roller_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
