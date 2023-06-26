def control_algorithm():
    from ..roller_control.control_algorithm import stanley_control
    steer, yaw_term, cte_term, min_dist, min_index = \
        stanley_control(0, 0, 0.3, 0, [0, 10], [0, 10], [0, 0])
    assert yaw_term == -0.3
