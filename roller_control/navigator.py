# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from statemachine import StateMachine, State
from std_msgs.msg import String

from .path_generator import PathGenerator


class VibrationRollerStateMachine(StateMachine):
    idle = State(initial=True)
    preparing_goal = State()
    navigating = State()

    plan_path = (
        idle.to(preparing_goal)
        | preparing_goal.to(preparing_goal)
    )
    go = preparing_goal.to(navigating)
    stop = navigating.to(idle)
    # navigation_done = navigating.to(idle)

    def __init__(self, nav):
        self.navigator :Navigator = nav
        super(VibrationRollerStateMachine, self).__init__()

    @property
    def navigator(self):
        return self._navigator

    @navigator.setter
    def navigator(self, nav):
        self._navigator = nav

    def on_enter_idle(self):
        print('idle state')

    def on_enter_preparing_goal(self):
        print('preparing_goal state')

    def on_enter_navigating(self):
        print('navigating state')

    def on_plan_path(self):
        print('Path planning started')
        self.navigator.plan_path()
        print('Path planning has been done')

    def on_go(self):
        print('Navigating has been started')
        self.navigator.go()

    def after_go(self):
        print('Navigating has been done')

    def on_stop(self):
        print('Stopping')
        self.navigator.stop()

    def after_stop(self):
        print('Vehicle stopped')


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')
        qos_profile = QoSProfile(depth=10)
        self.rollermotioncmd_subscriber = self.create_subscription(
            String,
            'roller_motion_cmd',
            self.recieve_motioncmd,
            qos_profile
        )

        self.sm = VibrationRollerStateMachine(self)

        self.basepoint = [371262.716, 159079.566]

    def recieve_motioncmd(self, msg: String):
        self.get_logger().info(f'{msg}')
        try:
            if msg.data == 'STOP' or msg.data =='E-STOP':
                self.sm.stop()
            elif 'PATHFILE' in msg.data:
                self.load_pathfile(msg.data)
            elif msg.data == 'PLAN PATH':
                self.sm.plan_path()
            elif msg.data == 'START MOTION':
                self.sm.go()
        except Exception as e:
            self.get_logger().warn(f'{e}')

    def load_pathfile(self, filenamecmd):
        _, filename = filenamecmd.split(':')
        with open(filename, 'r') as pathfile:
            self.path_json = json.load(pathfile)
        self.get_logger().info(f'Path file \'{filename.split("/")[-1]}\' has been loaded')
        self.get_logger().info(f'{self.path_json}')

    def plan_path(self):
        if self.path_json is None:
            self.get_logger().warn('select path file first')
            return
        ref_v = self.path_json['startPoint']['velocity']
        is_backward = ref_v < 0
        s_x = self.path_json['startPoint']['coordinate'][1] - self.basepoint[1]
        s_y = self.path_json['startPoint']['coordinate'][0] - self.basepoint[0]
        s_yaw = self.path_json['startPoint']['heading']
        g_x = self.path_json['endPoint']['coordinate'][1] - self.basepoint[1]
        g_y = self.path_json['endPoint']['coordinate'][0] - self.basepoint[0]
        g_yaw = self.path_json['endPoint']['heading']
        p = PathGenerator(s_x=s_x, s_y=s_y, s_yaw=s_yaw,
                        g_x=g_x, g_y=g_y, g_yaw=g_yaw,
                        ref_v=ref_v, is_backward=is_backward)
        self.map_xs, self.map_ys, self.map_yaws, self.cmd_vel = p.plan_path()

        for i in range(len(self.map_xs)):
            print(f'x:{self.map_xs[i] :.3f},'
                    f' y:{self.map_ys[i] :.3f},'
                    f' theta(deg):{self.map_yaws[i]/np.pi*180 :.3f},'
                    f' vel:{self.cmd_vel[i] :.2f}')
        self.get_logger().debug(f'{ref_v} {is_backward} {s_x :.3f} {s_y :.3f} {g_x :.3f} {g_y :.3f}')
        self.get_logger().info('path stamped has been loaded')

    def go(self):
        self.get_logger().info('motion started')
        self.get_logger().info('motion done')
        #     if self.map_xs is None\
        #         or self.map_ys is None\
        #         or self.map_yaws is None\
        #         or self.cmd_vel is None:
        #         self.get_logger().warn('path is empty')
        #         return

        #     if self.control_timer is None:
        #         self.control_timer = self.create_timer(CONTROL_PERIOD, self.control)
        #     else:
        #         self.get_logger().info('control algorithm is already running')
        #         return
        #     self.get_logger().info(f'Motion is started')

    def stop(self):
        self.get_logger().info('motion stopping')
        self.get_logger().info('motion stoped')
        # if msg.data == 'STOP' or msg.data =='E-STOP':
        #     if self.control_timer is not None:
        #         self.control_timer.cancel()
        #         self.control_timer = None
        #         self.get_logger().info('control algorithm has been stopped')
        #         return
        return

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
