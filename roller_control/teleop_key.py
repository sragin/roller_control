# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import sys

from geometry_msgs.msg import Twist
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QApplication, QLabel, QRadioButton, QWidget
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RollerTeleopKeyPublisher(QWidget):

    def __init__(self):
        super(RollerTeleopKeyPublisher, self).__init__()
        self.initUI()
        self.initROS()
        self.cmd_vel = Twist()
        self.cmd_motion = String()

        # self.timer_ = QTimer(self)
        # self.timer_.timeout.connect(self.publish_commands)
        # self.timer_.start(50)

    def initUI(self):
        self.setWindowTitle('Roller Teleop Key')
        str_ = 'Use arrow keys to move the roller.\n' \
            'Use "P" to read path file & generate path.\n' \
            'Use ";" to generate backward path.\n' \
            'Use "O" to run.\n'\
            'Use "I" or "S" to stop.'
        label = QLabel(str_, self)
        label.setGeometry(0, 0, 300, 150)
        label.setWordWrap(True)

        self.rbtn_manual = QRadioButton('Manual', self)
        self.rbtn_manual.setChecked(True)
        self.rbtn_auto = QRadioButton('Auto', self)
        self.rbtn_auto.move(70, 0)
        self.show()

        self.rbtn_manual.clicked.connect(self.clickMode)
        self.rbtn_auto.clicked.connect(self.clickMode)

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
        elif e.key() == Qt.Key.Key_P:
            self.cmd_motion.data = 'PATH'
        elif e.key() == Qt.Key.Key_Semicolon:
            self.cmd_motion.data = 'PATH_BACKWARD'
        elif e.key() == Qt.Key.Key_O:
            self.cmd_motion.data = 'START'
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

    def publish_commands(self):
        self.teloopcmd_publisher.publish(self.cmd_vel)
        if self.cmd_motion.data != '':
            self.motioncmd_publisher.publish(self.cmd_motion)
        # self.node.get_logger().info(f'Publishing: {msg}')


def main():
    app = QApplication(sys.argv)
    try:
        roller = RollerTeleopKeyPublisher()
    except Exception as e:
        roller.node.get_logger().info(f'{e}')
        roller.node.get_logger().info('Set all commands to zero')
        roller.node.destroy_node()
        rclpy.shutdown()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
