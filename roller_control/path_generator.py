import rclpy
from rclpy.node import Node
from roller_interfaces.srv import Path

import numpy as np


class PathGeneratorService(Node):
    def __init__(self):
        super().__init__('path_generator_service')
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.service = self.create_service(Path, 'roller_path', self.generate_path)

    def generate_path(self, request, response):
        waypoints = [
            (0, 0, 0.1),
            (20, 20, 0)
        ]
        xs = waypoints[0][0]
        xe = waypoints[1][0]
        ys = waypoints[0][1]
        ye = waypoints[1][1]
        cmd_vels = waypoints[0][2]
        cmd_vele = waypoints[1][2]

        dist = np.sqrt(pow(xe-xs, 2) + pow(ye-ys, 2))
        count = int(dist * 10)  # 0.1m 간격으로 목표점 인터폴레이션
        map_xs = np.linspace(xs, xe, count)
        map_ys = np.linspace(ys, ye, count)
        map_yaws = np.arctan(np.gradient(map_ys)/np.gradient(map_xs))
        cmd_vel = np.linspace(cmd_vels, cmd_vels, count)
        cmd_vel[-1] = cmd_vele

        response.x = map_xs
        response.y = map_ys
        response.theta = map_yaws
        response.vel = cmd_vel

        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        path_generator = PathGeneratorService()
        rclpy.spin(path_generator)
    except KeyboardInterrupt:
        path_generator.get_logger().info('Keyboard interrrupt (SIGINT)')
    finally:
        path_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
