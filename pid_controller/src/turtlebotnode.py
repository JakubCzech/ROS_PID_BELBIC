#!/usr/bin/env python

import rospy
import math

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from pid_controller import PID_CONTROLLER, PID_CONTROLLER_BELBIC
from geometry_msgs.msg import Twist
from time import perf_counter
from std_srvs.srv import Empty
from pd_controller import PD_CONTROLLER
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)


class TurtleBotNode:

    def __init__(self, max_time=20, acc=0.01):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.callback_update_position)
        self.pose = [0, 0]
        self.yaw_deg = 0
        self.error_fl = 0
        self.accuracy = acc
        self.max_time = max_time
        # self.rate = rospy.Rate(100)
        self.w = 0.0
        self.v = 0.0
        self.alfa = 0.00002
        self.beta = 0.00009

    def callback_update_position(self, data):
        self.pose[0] = round(data.pose.pose.position.x, 4)
        self.pose[1] = round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)

    def calculations(self, destination, pid, type):
        goal_pose = destination
        # PID_Yaw = PID_CONTROLLER(pid[0], pid[1], pid[2], 0.3)
        # PID_Distance= PID_CONTROLLER(pid[0], pid[1], pid[2], 0.5)
        if(type == "yaw"):
            PID_Yaw = PID_CONTROLLER(pid[0], pid[1], pid[2], 0.3)
            PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.2)
        elif(type == "distance"):
            PID_Distance = PID_CONTROLLER(pid[0], pid[1], pid[2], 0.2)
            PID_Yaw = PID_CONTROLLER(0.05, 0.00, 0.01, 0.3)
        # PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.2)
        # PID_Yaw = PID_CONTROLLER(0.05, 0.00, 0.01, 0.3)

        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        t0 = perf_counter()
        self.error_fl = False

        while distance > self.accuracy:
            psi = math.atan2(goal_pose[1] - self.pose[1], goal_pose[0] - self.pose[0])
            ang = math.degrees(psi)
            distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
            error_yaw = ang - self.yaw_deg
            out_yaw = PID_Yaw.set_current_error(error_yaw)
            out_distance = PID_Distance.set_current_error(distance)
            self.action(out_distance, out_yaw)
            t1 = perf_counter()
            if (t1-t0) > self.max_time:
                self.action(0, 0)
                self.error_fl = True
                break
        self.action(0, 0)

    def go_point_pos(self, goal_pose):
        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        PID_Yaw = PID_CONTROLLER(0.05, 0.00, 0.01, 0.3)
        PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.5)

        while distance > self.accuracy:
            if distance > 100.0:
                reset_simulation()
            psi = math.atan2(goal_pose[1] - self.pose[1], goal_pose[0] - self.pose[0])
            ang = math.degrees(psi)
            distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
            error_yaw = ang - self.yaw_deg
            out_yaw = PID_Yaw.set_current_error(error_yaw)
            out_distance = PID_Distance.set_current_error(distance)
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
        A = (self.v * SI)
        O = (self.w * SI)
        MO = (A - O)
        # Ath = (self.Vth * SI)
        # MO  = ((Ath + A) - O)
        rest = REW - A

        if rest < 0:
            rest = 0

        dv = self.alfa * (rest) * SI
        dw = self.beta * (MO - REW) * SI
        # dvth = self.alfa * (rest)     * SI
        self.v = self.v + dv
        self.w = self.w + dw
        # self.Vth = self.Vth + dvth
        U = MO
        if U > _limit_out:
            U = _limit_out
        elif U < -_limit_out:
            U = -_limit_out
        return U

    def calculations_belbic(self, destination, pid, type):  # destination(x,y),pid(p,i,d),test_type(yaw or distance)

        goal_pose = destination
        if(type == "yaw"):
            Cnt_Yaw_SI = PID_CONTROLLER_BELBIC(pid[0], pid[1], pid[2])
            Cnt_D_SI = PID_CONTROLLER_BELBIC(1.3, 1.8, 0.0001)
        elif(type == "distance"):
            Cnt_D_SI = PID_CONTROLLER_BELBIC(pid[0], pid[1], pid[2])
            Cnt_Yaw_SI = PID_CONTROLLER_BELBIC(0.02, 0.00, 0.00)

        Cnt_Yaw_SI = PID_CONTROLLER_BELBIC(0.02, 0.00, 0.00)
        Cnt_D_SI = PID_CONTROLLER_BELBIC(1.3, 1.8, 0.0001)

        Cnt_D_REW = PD_CONTROLLER(4.0, 7.5)
        Cnt_Yaw_REW = PD_CONTROLLER(0.02, 0.00)
        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        t0 = perf_counter()
        self.error_fl = False
        while distance >= self.accuracy:

            psi = math.atan2(goal_pose[1] - self.pose[1], goal_pose[0] - self.pose[0])
            ang = math.degrees(psi)
            distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
            error_yaw = ang - self.yaw_deg

            REW_D = Cnt_D_REW.set_REW(distance)
            REW_YAW = Cnt_Yaw_REW.set_REW(error_yaw)

            SI_D = Cnt_D_SI.set_SI(distance)
            SI_YAW = Cnt_Yaw_SI.set_SI(error_yaw)
            out_distance = self.Belbic(SI_D,   REW_D,    0.2)
            out_yaw = self.Belbic(SI_YAW, REW_YAW,  0.3)
            self.action(out_distance, out_yaw)
            t1 = perf_counter()
            if (t1-t0) > self.max_time or distance > 100.0:
                self.error_fl = True
                break
        self.action(0, 0)
