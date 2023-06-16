import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sys
from roller_interfaces.msg import RollerTeleop
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeyEvent


class RollerTeleopKeyPublisher(QWidget):
    def __init__(self):
        super(RollerTeleopKeyPublisher, self).__init__()
        self.initUI()
        self.initROS()
        self.timer_ = QTimer(self)
        self.timer_.timeout.connect(self.publish_commands)
        self.timer_.start(100)

    def initUI(self):
        self.setWindowTitle('Roller Teleop Key')
        str = "Use arrow keys to move the roller.\n" \
            "Use 'P' to read path file & generate path.\n" \
            "Use 'O' to run.\n"\
            "Use 'I' or 'S' to stop."
        label = QLabel(str, self)
        label.setGeometry(0, 0, 800, 600)
        label.setWordWrap(True)
        self.show()

    def initROS(self):
        rclpy.init(args=None)
        self.node = Node('roller_teleop_key')
        self.nodeName = self.node.get_name()
        self.motioncmd_publisher = self.node.create_publisher(
            String,
            'roller_motion_cmd',
            10)
        self.teloopcmd_publisher = self.node.create_publisher(
            RollerTeleop,
            'roller_teleop_cmd',
            10)
        self.command_ = [0, 0, 0]
        self.MAX_POWER = 20
        rclpy.spin_once(self.node, timeout_sec=1)

    def keyPressEvent(self, e: QKeyEvent) -> None:
        # self.node.get_logger().info(f'{e.key()} key pressed')
        if e.key() == Qt.Key.Key_Up:
            self.command_[2] = self.MAX_POWER
        elif e.key() == Qt.Key.Key_Down:
            self.command_[2] = -self.MAX_POWER
        elif e.key() == Qt.Key.Key_Left:
            self.command_[0] = self.MAX_POWER
        elif e.key() == Qt.Key.Key_Right:
            self.command_[1] = self.MAX_POWER
        elif e.key() == Qt.Key.Key_P:
            msg = String()
            msg.data = 'PATH'
            self.motioncmd_publisher.publish(msg)
        elif e.key() == Qt.Key.Key_O:
            msg = String()
            msg.data = 'START'
            self.motioncmd_publisher.publish(msg)
        elif e.key() == Qt.Key.Key_I or e.key() == Qt.Key.Key_S:
            msg = String()
            msg.data = 'STOP'
            self.motioncmd_publisher.publish(msg)
        return super().keyPressEvent(e)

    def keyReleaseEvent(self, a0: QKeyEvent) -> None:
        self.command_ = [0, 0, 0]
        return super().keyReleaseEvent(a0)

    def publish_commands(self):
        msg = RollerTeleop()
        msg.left_duty_control = self.command_[0]
        msg.right_duty_control = self.command_[1]
        msg.auto_spd_control = self.command_[2]

        self.teloopcmd_publisher.publish(msg)
        # self.node.get_logger().info(f'Publishing: {msg}')

def main():
    app = QApplication(sys.argv)
    try:
        roller = RollerTeleopKeyPublisher()
    except:
        msg = RollerTeleop()
        msg.left_duty_control = 0
        msg.right_duty_control = 0
        msg.auto_spd_control = 0
        roller.publisher_.publish(msg)
        roller.node.get_logger().info('Set all commands to zero')
        roller.node.get_logger().info('Keyboard interrupt (SIGINT)')
        roller.node.destroy_node()
        rclpy.shutdown()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
