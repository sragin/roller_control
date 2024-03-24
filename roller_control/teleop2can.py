from ament_index_python import get_package_share_directory
import cantools
from can_msgs.msg import Frame
import rclpy
from rclpy.node import Node
from teleop_msgs.msg import RollerTeleop


class Teleop2CanPublisher(Node):
    def __init__(self):
        super().__init__('teleop2can')
        self.get_logger().info(f'{self.get_name()} started')

        self.candb_controller = cantools.db.load_file(
            get_package_share_directory('roller_control') + '/Controller_230823.dbc')
        self.can_msg_control = self.candb_controller.get_message_by_name('CONTROLLER_COMM')
        self.candb_commandsv = cantools.db.load_file(
            get_package_share_directory('roller_control') + '/ToSupervisor_210430.dbc')
        self.can_msg_commandsv = self.candb_commandsv.get_message_by_name('Command_SV')
        self.subscriber_ = self.create_subscription(
            RollerTeleop,
            'teleop_cmd',
            self.recv_teleop_cmd,
            qos_profile=10
        )
        self.publisher_ = self.create_publisher(Frame, 'to_can_bus', qos_profile=10)
        CONTROL_PERIOD = 0.1
        self.create_timer(CONTROL_PERIOD, self.transmit_cancommand)

        self.msg = RollerTeleop()

        self.count = 0
        self.log_display_cnt = 10

    # 키보드나 조이스틱 조작으로 생성된 토픽(teleop_cmd/Rollerteleop 메시지)을
    # Autobox 송신용 캔 패킷으로 변환 후 socketcan용 토픽(to_can_bus/Frame 메시지)으로 송출한다
    def transmit_cancommand(self):
        mode = 1  # auto:1, manual:0
        commandsv_data = self.can_msg_commandsv.encode({
            'MODE': mode, 'AUTO_DRIVE': 0, 'STOP_CMD': 0})
        cmdsv_msg = Frame()
        cmdsv_msg.id = self.can_msg_commandsv.frame_id
        cmdsv_msg.dlc = self.can_msg_commandsv.length
        cmdsv_msg.data[:cmdsv_msg.dlc] = list(commandsv_data)
        self.publisher_.publish(cmdsv_msg)

        control_msg_data = self.can_msg_control.encode({
            'LEFT_DUTY_CONTROL': self.msg.steer_left,
            'RIGHT_DUTY_CONTROL': self.msg.steer_right,
            'AUTO_SPD_CONTROL': self.msg.drive,
            'VIB_MODE': 0,
            'VIB_CONTROL': 0,
            'DRV_MODE': 0,
            'HORN': 0,
            'RESERVED': 0})
        control_msg = Frame()
        control_msg.id = self.can_msg_control.frame_id
        control_msg.dlc = self.can_msg_control.length
        control_msg.data[:control_msg.dlc] = list(control_msg_data)
        self.publisher_.publish(control_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().warning(
                f'Control id: {control_msg.id} data: {control_msg.data}')
            self.get_logger().warning(
                f'Command id: {cmdsv_msg.id} data: {cmdsv_msg.data}')
            self.count = 0
        self.count += 1

    def recv_teleop_cmd(self, msg: RollerTeleop):
        self.msg = msg


def main(args=None):
    rclpy.init(args=args)
    try:
        teleop2can_publisher = Teleop2CanPublisher()
        rclpy.spin(teleop2can_publisher)
    except KeyboardInterrupt:
        teleop2can_publisher.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        teleop2can_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
