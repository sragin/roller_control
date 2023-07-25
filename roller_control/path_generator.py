# Copyright 2023 koceti_proprietary
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


import numpy as np


class PathGenerator:

    def __init__(self, s_x, s_y, s_yaw, g_x, g_y, g_yaw, s_v=0.25, ref_v=0.25, g_v=0.0, is_backward=False):
        self.s_x = s_x
        self.s_y = s_y
        self.s_yaw = s_yaw
        self.g_x = g_x
        self.g_y = g_y
        self.g_yaw = g_yaw
        self.s_v = s_v
        self.ref_v = ref_v
        self.g_v = g_v
        self.is_backward = is_backward
        # 롤러의 현재위치. 경로입력을 간단하게 하기 위해 사용
        self.x = 0
        self.y = 0
        # 경로생성 알고리즘 선택
        # self.plan_path = self.plan_simple_path
        # self.make_velocity_profile = self.make_simple_velocity_profile
        self.plan_path = self.plan_dubins_path
        self.make_velocity_profile = self.make_trapezoidal_velocity_profile

    def plan_simple_path(self):
        s_x = self.s_x + self.x
        g_x = self.g_x + self.x
        s_y = self.s_y + self.y
        g_y = self.g_y + self.y

        dist = np.sqrt(pow(g_x-s_x, 2) + pow(g_y-s_y, 2))
        count = int(dist * 10) + 1  # 0.1m 간격으로 목표점 인터폴레이션
        map_xs = np.linspace(s_x, g_x, count)
        map_ys = np.linspace(s_y, g_y, count)
        map_yaws = np.arctan2(np.gradient(map_ys), np.gradient(map_xs))

        cmd_vel = self.make_velocity_profile(self.s_v, self.ref_v, self.g_v, count)
        if self.is_backward:
            cmd_vel = [-v for v in cmd_vel]

        return map_xs, map_ys, map_yaws, cmd_vel

    def make_simple_velocity_profile(self, s_v, ref_v, g_v, count):
        vel_middle = [ref_v for _ in range(count - 10)]
        cmd_acc = [(i+1)*ref_v*0.1 for i in range(10)]
        cmd_dec = cmd_acc[::-1]
        return vel_middle + cmd_dec

    def plan_dubins_path(self):
        from .dubins_path import plan_dubins_path
        from .control_algorithm import MINIMUM_TURNING_RADIUS
        start_x = self.s_x
        start_y = self.s_y
        start_yaw = self.s_yaw

        end_x = self.g_x
        end_y = self.g_y
        end_yaw = self.g_yaw

        curvature = 1 / (MINIMUM_TURNING_RADIUS * 1.1)

        path_x, path_y, path_yaw, mode, lengths = \
            plan_dubins_path(start_x, start_y, start_yaw,
                             end_x, end_y, end_yaw,
                             curvature, 0.02)
        cmd_vel = self.make_velocity_profile(self.s_v, self.ref_v, self.g_v,
                                            path_x, path_y)
        if self.is_backward:
            cmd_vel = [-v for v in cmd_vel]
        cmd_vel[0] = cmd_vel[1]

        return path_x, path_y, path_yaw, cmd_vel

    def make_trapezoidal_velocity_profile(self, s_v, ref_v, g_v, path_x, path_y):
        from .control_algorithm import ACCELERATION

        dist_total = np.sqrt(pow(path_x[-1] - path_x[0], 2) + pow(path_y[-1] - path_y[0], 2))
        dist_acc = ref_v**2 / ACCELERATION / 2
        dist_dec = dist_total - dist_acc

        cmd_vel = []
        for i in range(len(path_x)):
            pos = np.sqrt(pow(path_x[i] - path_x[0], 2) + pow(path_y[i] - path_y[0], 2))
            if pos < dist_acc:
                vel = np.sqrt(2 * ACCELERATION * pos)
            elif pos < dist_dec:
                vel = ref_v
            else:
                vel = np.sqrt(2 * ACCELERATION * np.abs(dist_total-pos))
            cmd_vel.append(vel)
        return cmd_vel
