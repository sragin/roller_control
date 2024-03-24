import cantools
from can_msgs.msg import Frame
from rclpy.node import Node
from teleop_msgs.msg import Exteleop

DB_HYD = cantools.database.load_file('.//install//ex25_test_pkg//share//pattern_190909.dbc')
gVal_hydaccmd1 = DB_HYD.get_message_by_name('ToHYDAC_cmd_D_1')
gVal_hydaccmd2 = DB_HYD.get_message_by_name('ToHYDAC_cmd_D_2')


class Teleop2CanPublisher(Node):
    def __init__(self):
        super().__init__('teleop2can')

        self.can_msg_subscriber = self.create_subscription(
            Exteleop,
            'teleop_cmd',
            self.recv_teleop_cmd,
            qos_profile=10,
            callback_group=self.callback_group
        )
        self.publisher_ = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.count = 0
        self.log_display_cnt = 50

    # 키보드나 조이스틱 조작으로 생성된 토픽(ex25_teleop_cmd/Exteleop 메시지)을 
    # HYDAC 송신용 캔 패킷으로 변환 후 socketcan용 토픽(to_can_bus/Frame 메시지)으로 던진다
    def recv_teleop_cmd(self, msg):
        data = gVal_hydaccmd1.encode(
            {'Arm_in_H_D':msg.arm_in_h_d, 
             'Arm_out_H_D':msg.arm_out_h_d, 
             'Boom_up_H_D':msg.boom_up_h_d, 
             'Boom_dn_H_D':msg.boom_dn_h_d}
            )
        msg_eppr_cmd = Frame()
        msg_eppr_cmd.id = gVal_hydaccmd1.frame_id
        for i in range(6):
            msg_eppr_cmd.data[i] = int(data[i])
            
        msg_eppr_cmd.dlc = gVal_hydaccmd1.length
        self.publisher_.publish(msg_eppr_cmd)     
        
        data = gVal_hydaccmd2.encode(
            {'Bkt_in_H_D':msg.bkt_in_h_d, 
             'Bkt_out_H_D':msg.bkt_out_h_d, 
             'Swing_l_H_D':msg.swing_l_h_d, 
             'Swing_r_H_D':msg.swing_r_h_d}
            )
        msg_eppr_cmd2 = Frame()
        msg_eppr_cmd2.id = gVal_hydaccmd2.frame_id
        for i in range(6):
            msg_eppr_cmd2.data[i] = int(data[i])
            
        msg_eppr_cmd2.dlc = gVal_hydaccmd2.length
        self.publisher_.publish(msg_eppr_cmd2)     

        if self.count == self.log_display_cnt:
            self.get_logger().info(f"Received: {msg}")
            self.get_logger().warning(f"ToEPPR_cmd_D_1 id: {msg_eppr_cmd.id} data: {msg_eppr_cmd.data}")
            self.get_logger().warning(f"ToEPPR_cmd_D_2 id {msg_eppr_cmd2.id} data: {msg_eppr_cmd2.data}")
            self.count = 0
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    try:        
        exteleop2can_publisher = ExTeleop2CanPublisher()
        try:
            rclpy.spin(exteleop2can_publisher)
        except KeyboardInterrupt:
            exteleop2can_publisher.get_logger().info('Keyboard interrrupt (SIGINT)')
        finally:
            exteleop2can_publisher.destroy_node()           
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
