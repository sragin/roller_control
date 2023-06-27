from can_msgs.msg import Frame
import cantools
from msg_gps_interface.msg import GPSMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from roller_interfaces.msg import RollerStatus


class RollerPublisher(Node):

    def __init__(self):
        super().__init__('roller_publisher')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.candb_autobox_to_supervisor = \
            cantools.db.load_file('./install/roller_control/share/ToSupervisor_230619.dbc')
        self.can_msg_response = self.candb_autobox_to_supervisor.get_message_by_name('Response')
        self.can_msg_drum_pos = \
            self.candb_autobox_to_supervisor.get_message_by_name('Drum_Position')
        self.can_msg_drum_ori = \
            self.candb_autobox_to_supervisor.get_message_by_name('Drum_Orientation')
        self.can_msg_commandsv = self.candb_autobox_to_supervisor.get_message_by_name('Command_SV')

        self.can_msg_subscriber = self.create_subscription(
            Frame,
            'from_can_bus',
            self.recv_autobox_state,
            qos_profile
        )
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            qos_profile
        )
        self.canbus_publisher = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.roller_status_publisher = \
            self.create_publisher(RollerStatus, 'roller_status', qos_profile)

        self.timer_roller_geometry_msg = self.create_timer(1/50, self.publish_roller_geometry_msg)

        self.theta = 0.0  # radian. 바디의 헤딩각
        self.steer_angle = 0.0  # radian 스티어링 각도
        self.position = [0., 0.]  # position X(N), Y(E)
        self.response = [0, 0]
        self.speed = 0.0

        # 소부연 테스트베드 원점. 임의로 정한값임. 수준점 측량 후 변경해줘야 함)
        self.basepoint = [371394.576, 159245.570]
        # self.basepoint = [0.0, 0.0]

        self.count = 0
        self.log_display_cnt = 50

    def recv_autobox_state(self, msg):
        if msg.id == self.can_msg_response.frame_id:
            _cur = self.can_msg_response.decode(msg.data)
            self.response[0] = _cur['MODE']
            self.response[1] = _cur['STATUS']
            self.steer_angle = _cur['STEER_ANGLE'] / 180 * np.pi

    def recv_gpsmsg(self, msg: GPSMsg):
        self.position[0] = msg.tm_x - self.basepoint[0]
        self.position[1] = msg.tm_y - self.basepoint[1]
        self.theta = (-msg.heading + 90) / 180 * np.pi  # 헤딩각과 조향각 방향 맞춤. East를 0도 변경
        self.theta = normalize_angle(self.theta)
        self.speed = msg.speed * 1000 / 3600  # km/h - m/s 단위변환

    def publish_roller_geometry_msg(self):
        DRUM_LENGTH = 1405  # 드럼이 BX992 기준점보다 앞서있는 거리. mm 단위
        msg = RollerStatus()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steer_angle = self.steer_angle
        msg.pose.theta = self.theta
        # 롤러좌표를 기준으로 드럼좌표를 계산하여 보낸다
        msg.pose.x = self.position[0] + DRUM_LENGTH * np.sin(self.theta + self.steer_angle) / 1000
        msg.pose.y = self.position[1] + DRUM_LENGTH * np.cos(self.theta + self.steer_angle) / 1000
        msg.speed = self.speed
        self.roller_status_publisher.publish(msg)
        if self.count == self.log_display_cnt:
            self.get_logger().info(f'MODE: {self.response[0]}, STATUS:{self.response[1]},'
                                   f' STEER_ANGLE:{self.steer_angle :.2f}')
            self.get_logger().info(f'DRUM POS_X: {msg.pose.x :.4f},'
                                   f' POS_Y:{msg.pose.y :.4f},'
                                   f' HEAD:{self.theta :.2f}, SPEED:{self.speed :.2f}'
                                   f' POS X:{self.position[0] :.3f}, Y:{self.position[1] :.3f}')
            self.count = 0
        self.count += 1


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


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
