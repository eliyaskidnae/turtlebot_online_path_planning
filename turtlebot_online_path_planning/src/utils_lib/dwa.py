import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

class RobotType(Enum):
    circle = 1
    rectangle = 0

class Config:   
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.8# [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 30 * math.pi / 180.0  # [rad/s]
        # self.max_yaw_rate = 50.0 * math.pi / 180.0  # [rad/s]   
        self.max_accel = 0.3  # [m/ss]
        self.max_delta_yaw_rate = 30.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 2.0  # [s]
        self.to_goal_cost_gain = 0.05
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value




def dwa_control(x , goal, svc ):
    """
    Dynamic Window Approach control
    """
    config = Config()
    dw = calc_dynamic_window(x, config)
  
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, config.ob , svc)
    # print("u" , u)
    return u


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob , svc ):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
          
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config , svc)
            # print("ob_cost",ob_cost)
            # print("speed_cost",speed_cost)
            # print("to_goal_cost",to_goal_cost)
            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                # print(best_u)
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    # print("best_u",best_u)
    return best_u , best_trajectory

def calc_obstacle_cost(trajectory, ob, config , svc):
    """
    calc obstacle cost inf: collision
    """
    min_r = float("Inf")
    for trajectory_point in trajectory:
        distance = svc.min_dis_obstacle(trajectory_point[0:2])

        min_r = min(distance, min_r)
    
    # print("min_r_distance",min_r)
    if( min_r == float("Inf") ):
        # print("min_r",0)
        return 0
    
    elif config.robot_type == RobotType.circle:
        if min_r<= config.robot_radius:
            # print("min_r",float("Inf"))
            return float("Inf")
        else:
            # print("min_r",1/min_r)
            return 1.0 / min_r  # OK




    #     if svc.min_dis_obstacle(trajectory_point) < 0.5:
    #         return float("Inf")


    # ox = ob[:, 0]
    # oy = ob[:, 1]
    # dx = trajectory[:, 0] - ox[:, None]
    # dy = trajectory[:, 1] - oy[:, None]
    # r = np.hypot(dx, dy)

    # if config.robot_type == RobotType.rectangle:
    #     yaw = trajectory[:, 2]
    #     rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    #     rot = np.transpose(rot, [2, 0, 1])
    #     local_ob = ob[:, None] - trajectory[:, 0:2]
    #     local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    #     local_ob = np.array([local_ob @ x for x in rot])
    #     local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    #     upper_check = local_ob[:, 0] <= config.robot_length / 2
    #     right_check = local_ob[:, 1] <= config.robot_width / 2
    #     bottom_check = local_ob[:, 0] >= -config.robot_length / 2
    #     left_check = local_ob[:, 1] >= -config.robot_width / 2
    #     if (np.logical_and(np.logical_and(upper_check, right_check),
    #                        np.logical_and(bottom_check, left_check))).any():
    #         return float("Inf")
    # elif config.robot_type == RobotType.circle:
    #     if np.array(r <= config.robot_radius).any():
    #         return float("Inf")

    # min_r = np.min(r)
    # return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost

