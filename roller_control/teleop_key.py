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
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RollerTeleopKeyPublisher(QDialog):

    def __init__(self):
        super().__init__()
        self.initUI()
        self.initROS()
        self.cmd_vel = Twist()
        self.cmd_motion = String()

    def initUI(self):
        self.setWindowTitle('Roller Teleop Key')
        self.setMinimumSize(450, 250)
        str_ = 'Use arrow keys to move the roller.\n' \
            'Use "P" to read path file & generate path.\n' \
            'Use ";" to generate backward path.\n' \
            'Use "O" to run.\n'\
            'Use "I" or "S" to stop.'
        label = QLabel(str_, self)
        label.setWordWrap(True)
        font = QFont()
        font.setPointSize(14)
        label.setFont(font)

        self.rbtn_manual = QRadioButton('Manual', self)
        self.rbtn_manual.setChecked(True)
        self.rbtn_manual.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")
        self.rbtn_manual.setFont(font)
        self.rbtn_auto = QRadioButton('Auto', self)
        self.rbtn_auto.setStyleSheet(u"QRadioButton::indicator\n"
"{\n"
"	width : 30px;\n"
"	height : 30px;\n"
"}")
        self.rbtn_auto.setFont(font)
        self.pushButtonLoadPathfile = QPushButton('Load Pathfile', self)
        self.pushButtonLoadPathfile.setMinimumHeight(40)
        self.pushButtonLoadPathfile.setFont(font)
        self.pushButtonPlanPath = QPushButton('Plan Path', self)
        self.pushButtonPlanPath.setMaximumHeight(40)
        self.pushButtonPlanPath.setFont(font)

        layout2 = QHBoxLayout()
        layout2.addWidget(self.rbtn_manual)
        layout2.addWidget(self.rbtn_auto)
        hspacer = QSpacerItem(20, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        layout2.addSpacerItem(hspacer)

        layout3 = QHBoxLayout()
        layout3.addWidget(self.pushButtonLoadPathfile)
        layout3.addWidget(self.pushButtonPlanPath)

        layout1 = QVBoxLayout()
        layout1.addLayout(layout2)
        layout1.addLayout(layout3)
        layout1.addWidget(label)
        vspacer = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout1.addSpacerItem(vspacer)

        self.setLayout(layout1)
        self.show()

        self.rbtn_manual.clicked.connect(self.clickMode)
        self.rbtn_auto.clicked.connect(self.clickMode)
        self.pushButtonLoadPathfile.clicked.connect(self.clickLoadJSON)

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
        roller = RollerTeleopKeyPublisher()
    except Exception as e:
        roller.node.get_logger().info(f'{e}')
        roller.node.get_logger().info('Set all commands to zero')
        roller.node.destroy_node()
        rclpy.shutdown()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
