# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from ament_index_python import get_package_share_directory
import folium
from geometry_msgs.msg import Twist
from msg_gps_interface.msg import GPSMsg
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys
from threading import Thread

from .localui import *


class RollerControlUI(Node):

    def __init__(self, ui:Ui_Dialog):
        super().__init__('roller_gui')
        self.nodeName = self.get_name()
        self.get_logger().info(f'{self.nodeName} started')

        self.ui = ui
        self.initUI()
        self.cmd_vel = Twist()
        self.cmd_motion = String()

        # self.motioncmd_publisher = self.create_publisher(
        #     String,
        #     'roller_motion_cmd',
        #     QoSProfile(depth=10)
        # )
        # self.velocitycmd_publisher = self.create_publisher(
        #     Twist,
        #     'cmd_vel',
        #     QoSProfile(depth=10)
        # )
        self.gps_msg_subscriber = self.create_subscription(
            GPSMsg,
            'gps_msg',
            self.recv_gpsmsg,
            QoSProfile(depth=10)
        )

    def initUI(self):
        self.tiles = "http://mt0.google.com/vt/lyrs=y&hl=ko&x={x}&y={y}&z={z}"
        self.attr = "Google"
        latitude = 35.939219  # 위도
        longitude = 126.548906  # 경도
        self.map = folium.Map(
            location=[latitude, longitude],
            max_zoom=30, zoom_start=20,
            tiles=self.tiles, attr=self.attr,
            width=800, height=600
        )
        folium.Circle(location=[latitude, longitude], radius=1).add_to(self.map)
        self.w = self.ui.webEngineView
        self.w.setHtml(self.map.get_root().render())
        self.flag_webview_finished = False
        self.webview_timer = self.create_timer(1, self.update_webview)

        self.ui.radioButtonAuto.clicked.connect(self.clickMode)
        self.ui.radioButtonManual.clicked.connect(self.clickMode)
        self.ui.pushButtonLoadPathfile.clicked.connect(self.clickPlanning)
        self.ui.pushButtonPlanPath.clicked.connect(self.clickPlanning)
        self.ui.pushButtonPlanTask.clicked.connect(self.clickPlanning)
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
        self.ui.webEngineView.loadFinished.connect(self.webview_loadFinished)

    def clickMode(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        if self.ui.radioButtonAuto.isChecked():
            self.cmd_motion.data = 'AUTO'
        elif self.ui.radioButtonManual.isChecked():
            self.cmd_motion.data = 'MANUAL'
        self.publish_commands()
        self.velocitycmd_publisher.publish(self.cmd_vel)

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
        self.motioncmd_publisher.publish(self.cmd_motion)
        self.get_logger().info(f'Publishing: {self.cmd_motion.data}')

    def webview_loadFinished(self, ok):
        self.flag_webview_finished = ok
        # self.update_webview()
        self.webview_timer = self.create_timer(1, self.update_webview)

    def update_webview(self):
        print(self.flag_webview_finished, self.lat, self.lon)
        if self.flag_webview_finished:
            self.flag_webview_finished = False
            map = folium.Map(
                location=[self.lat, self.lon],
                max_zoom=30, zoom_start=20,
                tiles=self.tiles, attr=self.attr,
                width=800, height=600
            )
            folium.Circle(location=[self.lat, self.lon], radius=1).add_to(map)
            self.w.setHtml(map.get_root().render())
            # page = self.w.page()
            # page.runJavaScript(f"document.body.innerHTML = '{map.get_root().render()}'")
        self.webview_timer.cancel()


    def recv_gpsmsg(self, msg: GPSMsg):
        self.lat = msg.lat
        self.lon = msg.lon
        # self.get_logger().info(f'lat:{msg.lat:.6f} lng:{msg.lon:.6f}')

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    HMI = QDialog()
    ui = Ui_Dialog()
    ui.setupUi(HMI)

    hmi_node = RollerControlUI(ui)
    executer = MultiThreadedExecutor()
    executer.add_node(hmi_node)

    thread = Thread(target=executer.spin)
    thread.start()
    hmi_node.get_logger().info("Spinned GUI node . . .")

    try:
        HMI.show()
        sys.exit(app.exec_())
    finally:
        hmi_node.get_logger().info('Shutting down GUI node ...')
        hmi_node.destroy_node()
        executer.shutdown()


if __name__ == '__main__':
    main()
