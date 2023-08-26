# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import sys

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Twist
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .localui import *


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
        self.ui.pushButtonLoadPathfile.clicked.connect(self.clickPlanning)
        self.ui.pushButtonPlanPath.clicked.connect(self.clickPlanning)
        self.ui.pushButtonPlanTask.clicked.connect(self.clickPlanning)
        self.ui.pushButtonStartTask.clicked.connect(self.clickControlling)
        self.ui.pushButtonStop.clicked.connect(self.clickControlling)
        self.ui.pushButtonEStop.clicked.connect(self.clickControlling)
        self.ui.pushButtonRepeat.clicked.connect(self.clickControlling)
        self.ui.radioButtonVibrationOff.clicked.connect(self.clickVibration)
        self.ui.radioButtonVibrationHigh.clicked.connect(self.clickVibration)
        self.ui.radioButtonVibrationLow.clicked.connect(self.clickVibration)
        self.ui.radioButtonHornOff.clicked.connect(self.clickHorn)
        self.ui.radioButtonHornOn.clicked.connect(self.clickHorn)
        self.ui.radioButtonTravelForwardUphill.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelRabbit.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelRamp.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelReverseUphill.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelTurtle.clicked.connect(self.clickTravel)

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

    def clickMode(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        if self.ui.radioButtonAuto.isChecked():
            self.cmd_motion.data = 'AUTO'
        elif self.ui.radioButtonManual.isChecked():
            self.cmd_motion.data = 'MANUAL'
        self.publish_commands()

    def clickPlanning(self):
        button = self.sender()
        if button == self.ui.pushButtonLoadPathfile:
            filename, _ = QFileDialog.getOpenFileName(self, 'Open File', get_package_share_directory('roller_control'), filter='*.json')
            if filename == "":
                return
            self.cmd_motion.data = f'PATHFILE:{filename}'
        if button == self.ui.pushButtonPlanPath:
            self.cmd_motion.data = 'PLAN PATH'
        elif button == self.ui.pushButtonPlanTask:
            self.cmd_motion.data = 'PLAN TASK'
        self.publish_commands()

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
        elif button == self.ui.pushButtonRepeat:
            if self.ui.pushButtonRepeat.isChecked():
                self.cmd_motion.data = 'REPEAT ON'
            else:
                self.cmd_motion.data = 'REPEAT OFF'
        self.publish_commands()

    def clickVibration(self):
        button = self.sender()

        if button == self.ui.radioButtonVibrationHigh:
            self.cmd_motion.data = 'VIBRATION HIGH'
        elif button == self.ui.radioButtonVibrationLow:
            self.cmd_motion.data = 'VIBRATION LOW'
        else:
            self.cmd_motion.data = 'VIBRATION OFF'
        self.publish_commands()

    def clickHorn(self):
        button = self.sender()

        if button == self.ui.radioButtonHornOn:
            self.cmd_motion.data = 'HORN ON'
        else:
            self.cmd_motion.data = 'HORN OFF'
        self.publish_commands()

    def clickTravel(self):
        button = self.sender()

        if button == self.ui.radioButtonTravelRamp:
            self.cmd_motion.data = 'TRAVEL RAMP'
        elif button == self.ui.radioButtonTravelForwardUphill:
            self.cmd_motion.data = 'TRAVEL F UPHILL'
        elif button == self.ui.radioButtonTravelReverseUphill:
            self.cmd_motion.data = 'TRAVEL R UPHILL'
        elif button == self.ui.radioButtonTravelRabbit:
            self.cmd_motion.data = 'TRAVEL RABBIT'
        else:
            self.cmd_motion.data = 'TRAVEL TURTLE'
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
