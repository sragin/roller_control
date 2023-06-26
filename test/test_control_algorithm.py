def test_success():
    assert 1 == 1


def test_control_algorithm():
    import numpy as np
    from roller_control.control_algorithm import MAX_STEER_LIMIT
    from roller_control.control_algorithm import stanley_control
    from roller_control.path_generator import PathGenerator
    generator = PathGenerator()
    map_xs, map_ys, map_yaws, cmd_vel = generator.generate_path()
    steer_angle = 0.133
    theta = 0.191
    x = 10.489
    y = 3.092
    v = 0.5
    steer_, yaw_term_, cte_term_, min_dist_, min_index_ = \
        stanley_control(x, y, steer_angle + theta, v, map_xs, map_ys, map_yaws)
    steer_cmd = np.clip(steer_, -MAX_STEER_LIMIT, MAX_STEER_LIMIT)
    steer_cmd = steer_
    print(
        f'steer_:{steer_ :.3f}, yaw:{yaw_term_ :.3f}, cte:{cte_term_ :.3f},'
        f' min_dist:{min_dist_ :.3f} idx:{min_index_}\n'
        f'xs:{map_xs[0] :.3f} xe:{map_xs[-1] :.3f} x:{x :.3f} '
        f'ys:{map_ys[0] :.3f} ye:{map_ys[-1] :.3f} y:{y :.3f}\n'
        f'steer(rad):{steer_angle :.3f} steer_cmd(rad):{steer_cmd :.3f}'
        f' yaws:{map_yaws[0] :.3f} yaw:{theta + steer_angle :.3f}'
        f' cmd_vel:{cmd_vel[min_index_]}')
    assert yaw_term_ == -0.5 and steer_ == 0.200
