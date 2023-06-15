from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np


class PathGenerator():
    def __init__(self):
        self.waypoints = [
            (0, 0, 0.1),
            (20, 20, 0)
        ]

    def generate_path(self):
        xs = self.waypoints[0][0]
        xe = self.waypoints[1][0]
        ys = self.waypoints[0][1]
        ye = self.waypoints[1][1]
        cmd_vels = self.waypoints[0][2]
        cmd_vele = self.waypoints[1][2]

        dist = np.sqrt(pow(xe-xs, 2) + pow(ye-ys, 2))
        count = int(dist * 10)  # 0.1m 간격으로 목표점 인터폴레이션
        map_xs = np.linspace(xs, xe, count)
        map_ys = np.linspace(ys, ye, count)
        map_yaws = np.arctan(np.gradient(map_ys)/np.gradient(map_xs))
        cmd_vel = np.linspace(cmd_vels, cmd_vels, count)
        cmd_vel[-1] = cmd_vele

        path = Path()
        path.header.frame_id = 'world'

        for i in range(len(map_xs)):
            pose = PoseStamped()
            pose.pose.position.x = map_xs[i]
            pose.pose.position.y = map_ys[i]
            pose.pose.orientation = self.get_quaternion_from_euler(0, 0, map_yaws[i])
            path.poses.append(pose)

        return path


    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


def main(args=None):
    path_generator = PathGenerator()
    path_generator.generate_path()

if __name__ == '__main__':
    main()
