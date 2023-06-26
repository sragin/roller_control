import numpy as np

# ROLLER PARAMETERS
LENGTH = 5.870
LENGTH_REAR = 2.225
LENGTH_FRONT = 0.75
LENGTH_FRONT_TO_REAR = 2.975
WIDTH = 2.270
WHEEL_LEN = 0.75  # [m] 반지름
WHEEL_WIDTH_FRONT = 2.130  # [m]
WHEEL_WIDTH_REAR = 0.5  # [m]
TREAD_FRONT = 0  # [m]
TREAD_REAR = 0.85  # [m]
BACKTOWHEEL = LENGTH - WHEEL_LEN - 0.25  # from back to front wheel

MAX_STEER_VEL = 10.0 / 180 * np.pi
MAX_STEER = 31.5
MAX_STEER_LIMIT = 30


# stanley control algorithm
# x, y: meter, yaw: radian
def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws):
    # control gain
    k = 0.5

    # find the nearest point
    min_dist = 1e9
    min_index = 0
    n_points = len(map_xs)

    front_x = x + LENGTH_FRONT * np.cos(yaw)
    front_y = y + LENGTH_FRONT * np.sin(yaw)

    for i in range(n_points):
        dx = front_x - map_xs[i]
        dy = front_y - map_ys[i]

        dist = np.sqrt(dx * dx + dy * dy)
        if dist < min_dist:
            min_dist = dist
            min_index = i

    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]
    dx = map_x - front_x
    dy = map_y - front_y

    perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)

    # control law
    yaw_term = normalize_angle(map_yaw - yaw)
    cte_term = np.arctan2(k*cte, v)

    # steering
    steer = yaw_term + cte_term

    return steer, yaw_term, cte_term, min_dist, min_index


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle
