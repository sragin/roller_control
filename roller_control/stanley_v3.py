import math
import matplotlib.pyplot as plt
import numpy as np

# paramters
dt = 0.1

k = 10.0  # control gain

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

MAX_STEER_VEL = 30 * np.pi / 180  # rad/s (30 deg/s)
MAX_STEER = 31.5
MAX_STEER_LIMIT = 30


class VehicleModel(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw  # 차체의 헤딩 각도
        self.v = v
        self.w = 0
        self.steer = 0

        self.max_steering = np.radians(MAX_STEER_LIMIT)
        self.update = self.update_roller

    def update_roller(self, steer, a=0):
        steer = np.clip(steer, -self.max_steering, self.max_steering)
        self.steer = steer
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.w = self.v * steer / (LENGTH_FRONT + LENGTH_REAR) * dt
        self.yaw += (self.v * np.sin(steer) + LENGTH_REAR * self.w) * dt / (LENGTH_FRONT * np.cos(steer) + LENGTH_REAR)
        self.yaw = self.yaw % (2.0 * np.pi)
        self.v += a * dt


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


# x, y: meter, yaw: radian
def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws):
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
    return steer, yaw_term, cte_term, min_dist


# map
# 종료조건 개발필요
# - 오차범위 이내 접근하는 경우
# - 진행방향으로 지나쳤을 경우 (돌아올 수 없으므로) : 진행방향 좌표가 오차범위 이내(2.5cm)면 종료
def generate_path():
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

    # 경로가 사선일 때 - steady state error 존재 - goal yaw 각도를 넣어주니 해결됨
    # map_xs = np.linspace(0, 60, 500)
    # map_ys = np.linspace(0, 10, 500)
    # map_yaws = np.ones_like(map_xs) * get_angle(map_xs[0], map_ys[0], map_xs[-1], map_ys[-1])

    # 경로가 한 점일 때 - 지나친 후 이상동작 - 종료조건 필요
    # target_y = 5
    # map_xs = np.linspace(20, 21, 500)
    # map_ys = np.ones_like(map_xs) * target_y
    # map_yaws = np.ones_like(map_xs) * 0.0

    # 경로를 지나쳤을 때 - 전체 경로를 다 가지고 있을 경우에는 잘됨
    # target_y = 5
    # map_xs = np.linspace(-20, 60, 500)
    # map_ys = np.ones_like(map_xs) * target_y
    # map_yaws = np.ones_like(map_xs) * 0.0

    # 경로의 시작점이 앞에 있을 때 - 잘됨
    # target_y = 5
    # map_xs = np.linspace(20, 60, 500)
    # map_ys = np.ones_like(map_xs) * target_y
    # map_yaws = np.ones_like(map_xs) * 0.0

    # 사인파 경로
    # map_xs = np.linspace(0, 100, 500)
    # map_ys = 10 * np.sin(0.1 * map_xs) + 1
    # map_yaws = np.arctan(np.gradient(map_ys))

    # 원형경로
    # theta = np.linspace(0, 2 * np.pi, 500)
    # map_xs = 20 * np.cos(theta) - 20
    # map_ys = 20 * np.sin(theta)
    # map_yaws = np.pi / 2 + theta
    return map_xs, map_ys, map_yaws, cmd_vel


# return: radian
def get_angle(start_x, start_y, end_x, end_y):
    dy_ = end_y - start_y
    dx_ = end_x - start_x
    return np.math.atan2(dy_, dx_)


# vehicle
model = VehicleModel(x=0.0, y=0.0, yaw=np.pi/180*0, v=1.0)
steer = 0
map_xs, map_ys, map_yaws, cmd_vel = generate_path()

xs = []
ys = []
yaws = []
steers = []
ts = []
dxs, dys = [], []
for step in range(500):
    # plt.clf()
    t = step * dt

    steer_new, yaw_, cte_, min_dist_ = stanley_control(model.x, model.y, model.steer + model.yaw, model.v, map_xs, map_ys, map_yaws)
    steer_new = np.clip(steer_new, -model.max_steering, model.max_steering)
    if steer - steer_new >= MAX_STEER_VEL*dt:
        steer -= MAX_STEER_VEL*dt
    elif steer - steer_new <= -MAX_STEER_VEL*dt:
        steer += MAX_STEER_VEL*dt
    else:
        steer = steer_new
    model.update(steer)
    print(f"steer(deg):{steer * 180 / np.pi :.3f}, yaw:{yaw_ :.3f}, cte:{cte_ :.3f}, min_dist:{min_dist_ :.3f}")
    print(f"x:{model.x :.3f}, y:{model.y :.3f}")

    xs.append(model.x)
    ys.append(model.y)
    yaws.append(model.yaw)
    ts.append(t)
    steers.append(steer)

    # 종료조건 계산
    error = 0.05
    if map_xs[-1] > map_xs[0]:
        if map_xs[-1] - error <= model.x:
            break
    if map_ys[-1] > map_ys[0]:
        if map_ys[-1] - error <= model.y:
            break
    if map_xs[-1] < map_xs[0]:
        if map_xs[-1] - error >= model.x:
            break
    if map_ys[-1] < map_ys[0]:
        if map_ys[-1] - error >= model.y:
            break


# plot car
plt.figure(figsize=(12, 3))
plt.plot(map_xs, map_ys, 'r-', label="reference")
plt.plot(xs, ys, 'b--', alpha=0.5, label="stanley")
for i in range(len(xs)):
    # plt.clf()
    if i % 30 == 0:
        x = xs[i]
        y = ys[i]
        yaw = yaws[i]
        steer = steers[i]

        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                             [-WHEEL_WIDTH_FRONT / 2, -WHEEL_WIDTH_FRONT / 2, WHEEL_WIDTH_FRONT / 2, WHEEL_WIDTH_FRONT / 2, -WHEEL_WIDTH_FRONT / 2]])
        rr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                             [-WHEEL_WIDTH_REAR/2, -WHEEL_WIDTH_REAR/2, WHEEL_WIDTH_REAR/2, WHEEL_WIDTH_REAR/2, -WHEEL_WIDTH_REAR/2]])
        rl_wheel = np.copy(rr_wheel)

        Rot1 = np.array([[np.cos(yaw), np.sin(yaw)],
                         [-np.sin(yaw), np.cos(yaw)]])
        Rot2 = np.array([[np.cos(steer+yaw), np.sin(steer+yaw)],
                         [-np.sin(steer+yaw), np.cos(steer+yaw)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        # fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += 0 * np.cos(yaw) - TREAD_FRONT * np.sin(yaw)
        # fl_wheel[0, :] += L * np.cos(yaw) + TREAD * np.sin(yaw)
        fr_wheel[1, :] += 0 * np.sin(yaw) + TREAD_FRONT * np.cos(yaw)
        # fl_wheel[1, :] += L * np.sin(yaw) - TREAD * np.cos(yaw)
        rr_wheel[0, :] -= LENGTH_FRONT_TO_REAR
        rl_wheel[0, :] -= LENGTH_FRONT_TO_REAR
        rr_wheel[1, :] += TREAD_REAR
        rl_wheel[1, :] -= TREAD_REAR

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        plt.plot(x, y, "bo")
        # plt.plot(dx, dy, "ro")
        plt.axis("equal")
        # plt.pause(0.1)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(loc="best")
plt.tight_layout()
plt.savefig("stanley_method.png", dpi=300)
plt.show()
