import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from roller_interfaces.msg import RollerStatus
from std_msgs.msg import String

from .control_algorithm import MAX_STEER_LIMIT, MAX_STEER_VEL
import cantools
from can_msgs.msg import Frame
import numpy as np

CONTROL_PERIOD = 0.02


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.candb_controller = cantools.db.load_file('./install/roller_control/share/Controller_230518.dbc')
        self.can_msg_control = self.candb_controller.get_message_by_name('CONTROLLER_COMM')
        self.candb_commandsv = cantools.db.load_file('./install/roller_control/share/ToSupervisor_210430.dbc')
        self.can_msg_commandsv = self.candb_commandsv.get_message_by_name('Command_SV')

        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Frame, 'to_can_bus', qos_profile)
        self.cmdvel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.recieve_cmdvel,
            qos_profile
        )
        self.rollerstatus_subscriber = self.create_subscription(
            RollerStatus,
            'roller_status',
            self.recieve_rollerstatus,
            qos_profile
        )
        self.create_timer(CONTROL_PERIOD, self.velocity_controller)
        self.create_timer(CONTROL_PERIOD, self.steering_controller)
        self.create_timer(CONTROL_PERIOD, self.send_cancommand)

        self.cmd_drv_vel = 0.
        self.cmd_steer_vel = 0.
        self.out_velocity = 0.
        self.out_steering = 0.
        self.vel_pid = PID()
        self.vel_pid.SetTunnings(0.1, 0.0, 0.0)
        self.vel_pid.SetOutputLimits(1000.0)
        self.steer_pid = PID()
        self.steer_pid.SetTunnings(0.1, 0.0, 0.0)
        self.steer_pid.SetOutputLimits(100.0)

        self.roller_status = RollerStatus()
        self.last_steer = 0.

        self.count = 0
        self.log_display_cnt = 50

    def recieve_cmdvel(self, msg: Twist):
        self.cmd_drv_vel = msg.linear.x
        self.cmd_steer_vel = msg.angular.z

    def recieve_rollerstatus(self, msg: RollerStatus):
        self.roller_status = msg

    def velocity_controller(self):
        cur_velocity = 0
        out = self.vel_pid.Compute(self.cmd_drv_vel, cur_velocity)
        # self.get_logger().info(f'Velocity cmd: {self.cmd_drv_vel} out: {out}')
        self.out_velocity = out

    def steering_controller(self):
        cur_steer = self.roller_status.steer_angle.data
        steer_vel_deg = cur_steer - self.last_steer
        steer_vel = (steer_vel_deg / CONTROL_PERIOD) / 180 * np.pi
        out = self.steer_pid.Compute(self.cmd_steer_vel, steer_vel)
        self.out_steering = out

        self.last_steer = cur_steer
        self.get_logger().info(f'Steering cmd: {self.cmd_steer_vel :.2f} out: {out :.2f} cur: {steer_vel :.2f}')

    def send_cancommand(self):
        commandsv = self.can_msg_commandsv.encode(
            {'MODE':1,
             'AUTO_DRIVE':0,
             'STOP_CMD':0
             }
        )
        cmdsv_msg = Frame()
        cmdsv_msg.id = self.can_msg_commandsv.frame_id
        cmdsv_msg.dlc = self.can_msg_commandsv.length
        for i in range(cmdsv_msg.dlc):
            cmdsv_msg.data[i] = int(commandsv[i])
        self.publisher_.publish(cmdsv_msg)

        if self.out_steering > 0:
            left = self.out_steering
            right = 0.
        else:
            left = 0.
            right = -self.out_steering
        data = self.can_msg_control.encode(
            {'LEFT_DUTY_CONTROL':left,
             'RIGHT_DUTY_CONTROL':right,
             'AUTO_SPD_CONTROL':self.out_velocity,
             'UNUSED_CONTROL':0}
            )
        control_msg = Frame()
        control_msg.id = self.can_msg_control.frame_id
        for i in range(8):
            control_msg.data[i] = int(data[i])

        control_msg.dlc = self.can_msg_control.length
        self.publisher_.publish(control_msg)

        if self.count == self.log_display_cnt:
            self.get_logger().warning(f'Controller Command id: {control_msg.id} data: {control_msg.data}')
            self.get_logger().warning(f'Supervisor Command id: {cmdsv_msg.id} data: {cmdsv_msg.data}')
            self.get_logger().info(f'Velocity cmd: {self.cmd_drv_vel} out: {self.out_velocity}')
            self.get_logger().info(f'Steering cmd: {self.cmd_steer_vel} out: {self.out_steering}')
            self.count = 0
        self.count += 1


class PID():
    def __init__(self):
        self.lastInput = 0.
        self.errSum = 0.
        self.lastErr = 0.
        self.ITerm = 0.

        self.kp = 0.1
        self.ki = 0.
        self.kd = 0.

        self.outMax = 1.
        self.outMin = 0.

    def Compute(self, Setpoint, Input):
        # Compute all the working error variables
        error = Setpoint - Input
        self.ITerm += (self.ki * error)
        if self.ITerm > self.outMax:
            self.ITerm = self.outMax
        elif self.ITerm < self.outMin:
            self.ITerm = self.outMin
        dInput = (Input - self.lastInput)

        # Compute PID Output
        Output = self.kp * error + self.ITerm - self.kd * dInput
        if Output > self.outMax:
            Output = self.outMax
        elif Output < self.outMin:
            Output = self.outMin

        self.lastErr = error
        return Output

    def SetTunnings(self, Kp, Ki, Kd):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd

    def SetOutputLimits(self, Max, Min=0.0):
        if Min > Max:
            return
        self.outMin = Min
        self.outMax = Max


def main(args=None):
    rclpy.init(args=args)
    try:
        base_controller = BaseController()
        rclpy.spin(base_controller)
    except KeyboardInterrupt:
        base_controller.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        base_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
