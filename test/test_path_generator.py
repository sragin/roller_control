import pytest
from roller_control.path_generator import PathGenerator


@pytest.fixture
def p():
    p = PathGenerator(s_x=0, g_x=10, s_yaw=0, s_y=0, g_y=0, g_yaw=0, ref_v=1.25)
    return p


def test_success():
    assert 1 == 1


def test_map_1(p: PathGenerator):
    p.plan_path = p.plan_simple_path
    p.make_velocity_profile = p.make_simple_velocity_profile
    p.x = 5.474
    p.y = 14.711
    map_xs, map_ys, map_yaws, cmd_vel = p.plan_path()
    assert 5.474 == map_xs[0]
    assert 14.711 == map_ys[0]
    assert 1.25 == cmd_vel[0]


def test_dubins(p: PathGenerator):
    p.plan_path = p.plan_dubins_path
    p.make_velocity_profile = p.make_trapezoidal_velocity_profile
    path_x, path_y, path_yaw, cmd_vel = p.plan_path()
    assert 0 == path_x[0]
    # assert 0.1 == path_x[1]
    assert 0 == path_y[0]
    assert 0 == path_yaw[0]
    assert 10 == path_x[-1]
    assert 0 == cmd_vel[0]
    assert 0 == cmd_vel[-1]
    assert 1.25 == max(cmd_vel)
    # assert 1.0 == max(cmd_vel)
