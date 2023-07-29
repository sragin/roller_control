# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import sys

from geometry_msgs.msg import Twist
import json
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from .localui import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RollerControlUI(QDialog):

    def __init__(self):
        super().__init__()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        # self.initUI()
        self.initROS()
        self.cmd_vel = Twist()
        self.cmd_motion = String()

    def initUI(self):
        # self.rbtn_manual.clicked.connect(self.clickMode)
        # self.rbtn_auto.clicked.connect(self.clickMode)
        self.ui.pushButtonLoadPathfile.clicked.connect(self.clickLoadJSON)

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
        if self.rbtn_auto.isChecked():
            self.cmd_motion.data = 'AUTO'
        elif self.rbtn_manual.isChecked():
            self.cmd_motion.data = 'MANUAL'
        self.publish_commands()

    def clickLoadJSON(self):
        filename = './install/roller_control/share/path1.json'
        with open(filename, 'r') as path_json:
            path = json.load(path_json)
            # print(path['startPoint']['coordinate'])

    def publish_commands(self):
        self.teloopcmd_publisher.publish(self.cmd_vel)
        if self.cmd_motion.data != '':
            self.motioncmd_publisher.publish(self.cmd_motion)
        # self.node.get_logger().info(f'Publishing: {msg}')


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
