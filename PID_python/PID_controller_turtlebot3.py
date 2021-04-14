#!/usr/bin/env python
"""
Author : David Valencia
Date : March-29-2020

Description: Read basic info from turtlebot3 robot using ROS melodic and gazebo
             control position with a PID controller
"""
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from homogeneous_matrix import *


class TurtleBotNode:

    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.callback_update_position)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.pose_x = 0
        self.pose_y = 0
        self.yaw_deg = 0
        self.rate = rospy.Rate(10)

    def callback_update_position(self, data):
        self.pose_x = round(data.pose.pose.position.x, 4)
        self.pose_y = round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation

        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)

    def calculations(self):
        goal_pose_x = input("Set your x goal: ")
        goal_pose_y = input("Set your y goal: ")


        goal_pose_x = float(goal_pose_x)
        goal_pose_y = float(goal_pose_y)

        PID_Yaw = pid_controller(0.05, 0.00, 0.01, 0.3)
        PID_Distance = pid_controller(0.001, 0.1, 1.6, 0.5)

        distance = math.sqrt(math.pow((goal_pose_x - self.pose_x), 2) + math.pow((goal_pose_y - self.pose_y), 2))

        while distance >= 0.1:

            psi = math.atan2(goal_pose_y - self.pose_y, goal_pose_x - self.pose_x)
            ang = math.degrees(psi)
            print (ang)
            distance = math.sqrt(math.pow((goal_pose_x - self.pose_x), 2) + math.pow((goal_pose_y - self.pose_y), 2))
            error_yaw = ang - self.yaw_deg
            out_yaw = PID_Yaw.set_current_error(error_yaw)
            out_distance = PID_Distance.set_current_error(distance)
            self.action(out_distance, out_yaw)

        print("Reach to the desired point")
        self.action(0, 0)

    def action(self, U1, U2):
        twist = Twist()
        twist.linear.x = U1
        twist.angular.z = U2
        self.velocity_publisher.publish(twist)


class pid_controller:

    def __init__(self, p_coef, i_coef, d_coef, limit_out):

        self.kp = p_coef
        self.ki = i_coef
        self.kd = d_coef
        self._limit_out = limit_out
        self._previous_error = 0.0

    def set_current_error(self, error):

        output0 = error * self.kp

        error_diff = error - self._previous_error
        outpu1 = self.kd * error_diff

        error_intr = error + self._previous_error
        outpu2 = self.ki * error_intr

        self._previous_error = error

        output = output0 + outpu1 + outpu2

        if output > self._limit_out:
            output = self._limit_out
        elif output < (-self._limit_out):
            output = (-self._limit_out)

        return output


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":

    rospy.init_node('turtle_bot_basic_controller', anonymous=True)
    a = TurtleBotNode()
    a.calculations()
    rospy.on_shutdown(shutdown_callback)
    rospy.spin()
