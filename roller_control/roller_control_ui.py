# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import json
import sys

from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Twist
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import String

from .localui import *


class Ros2Node(Node):

    def __init__(self):
        super().__init__('roller_gui')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.motioncmd_publisher = self.create_publisher(
            String,
            'roller_motion_cmd',
            10)
        self.velocitycmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

class RollerControlUI(QDialog):

    def __init__(self, ros2_node: Node):
        super().__init__()
        self.node = ros2_node
        self.initUI()
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
        self.ui.pushButtonStartMotion.clicked.connect(self.clickControlling)
        self.ui.pushButtonStartTask.clicked.connect(self.clickControlling)
        self.ui.pushButtonStop.clicked.connect(self.clickControlling)
        self.ui.pushButtonEStop.clicked.connect(self.clickControlling)
        self.ui.pushButtonRepeat.clicked.connect(self.clickControlling)
        self.ui.radioButtonVibrationOff.clicked.connect(self.clickVibrationMode)
        self.ui.radioButtonVibrationHigh.clicked.connect(self.clickVibrationMode)
        self.ui.radioButtonVibrationLow.clicked.connect(self.clickVibrationMode)
        self.ui.pushButtonVibrationON.pressed.connect(self.clickVibrationOn)
        self.ui.pushButtonVibrationON.released.connect(self.clickVibrationOn)
        self.ui.pushButtonHornOn.pressed.connect(self.clickHorn)
        self.ui.pushButtonHornOn.released.connect(self.clickHorn)
        self.ui.radioButtonTravelForwardUphill.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelRabbit.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelRamp.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelReverseUphill.clicked.connect(self.clickTravel)
        self.ui.radioButtonTravelTurtle.clicked.connect(self.clickTravel)

    def clickMode(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        if self.ui.radioButtonAuto.isChecked():
            self.cmd_motion.data = 'AUTO'
        elif self.ui.radioButtonManual.isChecked():
            self.cmd_motion.data = 'MANUAL'
        self.publish_commands()
        self.node.velocitycmd_publisher.publish(self.cmd_vel)

    def clickPlanning(self):
        button = self.sender()
        if button == self.ui.pushButtonLoadPathfile:
            filename, _ = QFileDialog.getOpenFileName(self, 'Open File', get_package_share_directory('roller_control'), filter='*.json')
            if filename == "":
                return
            with open(filename, 'r') as f:
                data = f.read()
                try:
                    jd = json.loads(data)
                    self.node.get_logger().info(f'json file loaded {filename}')
                except Exception as e:
                    self.node.get_logger().error(f'failed to load json file. error:{e}')
                    return
                self.cmd_motion.data = 'PATHFILE:' + data
        if button == self.ui.pushButtonPlanPath:
            self.cmd_motion.data = 'PLAN PATH'
        elif button == self.ui.pushButtonPlanTask:
            self.cmd_motion.data = 'PLAN TASK'
        self.publish_commands()

    def clickControlling(self):
        button = self.sender()

        if button == self.ui.pushButtonStartMotion:
            self.cmd_motion.data = 'START MOTION'
        elif button == self.ui.pushButtonStartTask:
            self.cmd_motion.data = 'START TASK'
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

    def clickVibrationMode(self):
        button = self.sender()

        if button == self.ui.radioButtonVibrationHigh:
            self.cmd_motion.data = 'VIBRATION HIGH'
        elif button == self.ui.radioButtonVibrationLow:
            self.cmd_motion.data = 'VIBRATION LOW'
        else:
            self.cmd_motion.data = 'VIBRATION OFF'
        self.publish_commands()

    def clickVibrationOn(self):
        button = self.sender()

        if button.isDown() == True:
            self.cmd_motion.data = 'VIBRATION BTN PRESSED'
        else:
            self.cmd_motion.data = 'VIBRATION BTN RELEASED'
        self.publish_commands()

    def clickHorn(self):
        button = self.sender()

        if button.isDown() == True:
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
        self.node.motioncmd_publisher.publish(self.cmd_motion)
        self.node.get_logger().info(f'Publishing: {self.cmd_motion.data}')

def main():
    app = QApplication(sys.argv)
    rclpy.init(args=None)

    ros2_node = Ros2Node()
    gui_app = RollerControlUI(ros2_node)
    gui_app.show()

    while rclpy.ok():
        app.processEvents()
        rclpy.spin_once(ros2_node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
