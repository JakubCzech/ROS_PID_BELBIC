#!/usr/bin/env python

import math
from time import perf_counter
from typing import Tuple

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simple_pid import PID
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

from controllers import PD_Controller, PID_Belbic
from errors import NavigationMaxTimeError

reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
SPEED_LIMIT = 0.25


def log_info(func):
    def wrapper(*args, **kwargs):
        rospy.logdebug(f"Start {func.__name__}")
        rospy.logdebug(f"Args: {args}")
        result = func(*args, **kwargs)
        rospy.logdebug(f"End {func.__name__}")
        return result

    return wrapper


def calc_distance(
    p_start: Tuple[float, float], p_end: Tuple[float, float], acc: int = 4
) -> float:
    return round(
        math.sqrt(pow((p_start[0] - p_end[0]), 2) + pow((p_start[1] - p_end[1]), 2)),
        acc,
    )


class TurtleBotNode:
    def __init__(self, max_time: int = 20, acc: float = 0.01):
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.callback_update_position)

        self.pose: list[float, float] = [0, 0]
        self.yaw_deg: float = 0
        self.accuracy: float = acc
        self.max_time: int = max_time
        self.w: float = 0.0
        self.v: float = 0.0
        self.alfa: float = 0.00002
        self.beta: float = 0.00009

    def callback_update_position(self, data):
        self.pose[0] = round(data.pose.pose.position.x, 4)
        self.pose[1] = round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)

    @log_info
    def calculations(
        self, destination: Tuple[float, float], pid: Tuple[float, float, float]
    ) -> Tuple[float, float]:

        pid_linear = PID(pid[0], pid[1], pid[2], setpoint=0)
        pid_angular = PID(0.05, 0.00, 0.01, setpoint=0)

        distance = calc_distance(self.pose, destination)

        pid_linear.output_limits = (-SPEED_LIMIT, SPEED_LIMIT)
        pid_angular.output_limits = (-SPEED_LIMIT, SPEED_LIMIT)

        start_time = perf_counter()

        tmp_distance = distance
        while tmp_distance > self.accuracy and not rospy.is_shutdown():

            psi = math.atan2(
                destination[1] - self.pose[1], destination[0] - self.pose[0]
            )
            ang = math.degrees(psi)
            tmp_distance = calc_distance(self.pose, destination)
            error_yaw = ang - self.yaw_deg
            out_yaw = pid_angular(error_yaw)
            out_distance = pid_linear(tmp_distance)

            self.action(out_distance, out_yaw)
            if (perf_counter() - start_time) > self.max_time:
                self.action(0, 0)
                raise NavigationMaxTimeError("Time limit exceeded")

        return distance, perf_counter() - start_time

    def go_point_pos(self, goal_pose):
        distance = calc_distance(self.pose, goal_pose)
        pid_yaw = PID(0.05, 0.00, 0.01, setpoint=1)
        pid_distance = PID(0.001, 0.1, 1.6, setpoint=1)

        while distance > self.accuracy and not rospy.is_shutdown():
            if distance > 100.0:
                reset_simulation()
            psi = math.atan2(goal_pose[1] - self.pose[1], goal_pose[0] - self.pose[0])
            ang = math.degrees(psi)
            distance = calc_distance(self.pose, goal_pose)
            error_yaw = ang - self.yaw_deg
            out_yaw = pid_yaw(error_yaw)
            out_distance = pid_distance(distance)
            self.action(out_distance, out_yaw)
        self.action(0, 0)
        return True

    def action(self, U1, U2):
        twist = Twist()
        twist.linear.x = U1
        twist.angular.z = U2
        self.velocity_publisher.publish(twist)

    def Belbic(self, SI, REW, limite):
        _limit_out = limite
        A = self.v * SI
        O = self.w * SI
        MO = A - O
        rest = REW - A
        if rest < 0:
            rest = 0
        dv = self.alfa * (rest) * SI
        dw = self.beta * (MO - REW) * SI
        self.v = self.v + dv
        self.w = self.w + dw
        U = MO
        if U > _limit_out:
            U = _limit_out
        elif U < -_limit_out:
            U = -_limit_out
        return U

    def calculations_belbic(
        self, destination: Tuple[float, float], pid: Tuple[float, float, float]
    ) -> Tuple[float, float]:

        Cnt_D_SI = PID_Belbic(pid[0], pid[1], pid[2])
        Cnt_Yaw_SI = PID_Belbic(0.02, 0.00, 0.00)

        # Cnt_Yaw_SI = PID_Belbic(0.02, 0.00, 0.00)
        # Cnt_D_SI = PID_Belbic(1.3, 1.8, 0.0001)

        Cnt_D_REW = PD_Controller(4.0, 7.5)
        Cnt_Yaw_REW = PD_Controller(0.02, 0.00)
        distance = calc_distance(self.pose, destination)
        t0 = perf_counter()
        tmp_distance = distance
        while tmp_distance >= self.accuracy and not rospy.is_shutdown():
            psi = math.atan2(
                destination[1] - self.pose[1], destination[0] - self.pose[0]
            )
            ang = math.degrees(psi)
            distance = calc_distance(self.pose, destination)
            error_yaw = ang - self.yaw_deg

            REW_D = Cnt_D_REW.set_REW(distance)
            REW_YAW = Cnt_Yaw_REW.set_REW(error_yaw)
            SI_D = Cnt_D_SI.set_SI(distance)
            SI_YAW = Cnt_Yaw_SI.set_SI(error_yaw)

            out_distance = self.Belbic(SI_D, REW_D, 0.2)
            out_yaw = self.Belbic(SI_YAW, REW_YAW, 0.3)
            self.action(out_distance, out_yaw)

            if (perf_counter() - t0) > self.max_time or distance > 100.0:
                self.action(0, 0)
                raise NavigationMaxTimeError("Time limit exceeded")
        return distance, perf_counter() - t0
