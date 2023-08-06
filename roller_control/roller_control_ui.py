# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import json
import sys

from .localui import *
from .path_generator import PathGenerator
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Twist
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RollerControlUI(QDialog):

    def __init__(self):
        super().__init__()
        self.initUI()
        self.initROS()
        self.cmd_vel = Twist()
        self.cmd_motion = String()

    def initUI(self):
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.show()
        self.ui.radioButtonAuto.clicked.connect(self.clickMode)
        self.ui.radioButtonManual.clicked.connect(self.clickMode)
        self.ui.pushButtonLoadPathfile.clicked.connect(self.clickLoadJSON)
        self.ui.pushButtonPlanPath.clicked.connect(self.clickPlanPath)
        self.ui.pushButtonPlanTask.clicked.connect(self.clickPlanTask)
        self.ui.pushButtonStartTask.clicked.connect(self.clickControlling)
        self.ui.pushButtonStop.clicked.connect(self.clickControlling)
        self.ui.pushButtonEStop.clicked.connect(self.clickControlling)

    def initROS(self):
        rclpy.init(args=None)
        self.node = Node('roller_teleop_key')
        self.nodeName = self.node.get_name()
        self.motioncmd_publisher = self.node.create_publisher(
            String,
            'roller_motion_cmd',
            10)
        self.teloopcmd_publisher = self.node.create_publisher(
            Twist,
            'cmd_vel',
            10)
        rclpy.spin_once(self.node, timeout_sec=1)

    def keyPressEvent(self, e: QKeyEvent) -> None:
        # self.node.get_logger().info(f'{e.key()} key pressed')
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_motion.data = ''
        if e.key() == Qt.Key.Key_Up:
            self.cmd_vel.linear.x = 0.3
        elif e.key() == Qt.Key.Key_Down:
            self.cmd_vel.linear.x = -0.1
        elif e.key() == Qt.Key.Key_Left:
            self.cmd_vel.angular.z = 1.0
        elif e.key() == Qt.Key.Key_Right:
            self.cmd_vel.angular.z = -1.0
        elif e.key() == Qt.Key.Key_I or e.key() == Qt.Key.Key_S:
            self.cmd_motion.data = 'STOP'
        self.publish_commands()
        return super().keyPressEvent(e)

    def keyReleaseEvent(self, a0: QKeyEvent) -> None:
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_motion.data = ''
        self.publish_commands()
        return super().keyReleaseEvent(a0)

    def clickMode(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        if self.ui.radioButtonAuto.isChecked():
            self.cmd_motion.data = 'AUTO'
        elif self.ui.radioButtonManual.isChecked():
            self.cmd_motion.data = 'MANUAL'
        self.publish_commands()

    def clickLoadJSON(self):
        filename = get_package_share_directory('roller_control') + '/path.json'
        with open(filename, 'r') as pathfile:
            self.path_json = json.load(pathfile)
            # print(self.path_json['startPoint']['coordinate'])
        self.node.get_logger().info('Json path file has been loaded')

    def clickPlanPath(self):
        ref_v = self.path_json['startPoint']['velocity']
        is_backward = ref_v < 0
        s_x = self.path_json['startPoint']['coordinate'][0]
        s_y = self.path_json['startPoint']['coordinate'][1]
        s_yaw = self.path_json['startPoint']['heading']
        g_x = self.path_json['endPoint']['coordinate'][0]
        g_y = self.path_json['endPoint']['coordinate'][1]
        g_yaw = self.path_json['endPoint']['heading']
        p = PathGenerator(s_x=s_x, s_y=s_y, s_yaw=s_yaw,
                          g_x=g_x, g_y=g_y, g_yaw=g_yaw,
                          ref_v=ref_v, is_backward=is_backward)
        map_xs, map_ys, map_yaws, cmd_vel = p.plan_path()
        self.node.get_logger().info(f'{ref_v} {is_backward}')

        import numpy as np
        for i in range(len(map_xs)):
            print(f'x:{map_xs[i] :.3f},'
                    f' y:{map_ys[i] :.3f},'
                    f' theta(deg):{map_yaws[i]/np.pi*180 :.3f},'
                    f' vel:{cmd_vel[i] :.2f}')
        self.node.get_logger().info('path stamped has been loaded')

    def clickPlanTask(self):
        pass

    def clickControlling(self):
        button = self.sender()

        if button == self.ui.pushButtonStartTask:
            if self.ui.pushButtonRepeat.isChecked():
                self.cmd_motion.data = 'START TASK'
            else:
                self.cmd_motion.data = 'START MOTION'
        elif button == self.ui.pushButtonStop:
            self.cmd_motion.data = 'STOP'
        elif button == self.ui.pushButtonEStop:
            self.cmd_motion.data = 'E-STOP'
        self.publish_commands()

    def publish_commands(self):
        self.teloopcmd_publisher.publish(self.cmd_vel)
        if self.cmd_motion.data != '':
            self.motioncmd_publisher.publish(self.cmd_motion)
            self.node.get_logger().info(f'Publishing: {self.cmd_motion.data}')


def main():
    app = QApplication(sys.argv)
    try:
        roller = RollerControlUI()
    except Exception as e:
        roller.node.get_logger().info(f'{e}')
        roller.node.get_logger().info('Set all commands to zero')
        roller.node.destroy_node()
        rclpy.shutdown()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
