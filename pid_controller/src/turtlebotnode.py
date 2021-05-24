#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry
from pid_controller import PID_CONTROLLER
from geometry_msgs.msg import Twist
from time import perf_counter


class TurtleBotNode:
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.callback_update_position)
        self.pose_x = 1
        self.pose_y = 0
        self.yaw_deg = 0
        self.error_fl=0
        self.rate = rospy.Rate(10)
    def callback_update_position(self, data):
        self.pose_x = round(data.pose.pose.position.x, 4)
        self.pose_y = round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation

        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)
    def calculations(self,my_x,my_y,my_p, my_i, my_d):
        goal_pose_x = float(my_x)
        goal_pose_y = float(my_y)
        accuracy = 0.001
        if(my_x == 0.0 and my_y == 0.0):
            accuracy = 0.00000000000001

        PID_Yaw = PID_CONTROLLER(my_p, my_i, my_d, 0.3)
        PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.5)
        distance = math.sqrt(math.pow((goal_pose_x - self.pose_x), 2) + math.pow((goal_pose_y - self.pose_y), 2))
        t0 = perf_counter()
        self.error_fl=False

        while distance >= 0.01:

            psi = math.atan2(goal_pose_y - self.pose_y, goal_pose_x - self.pose_x)
            ang = math.degrees(psi)
            distance = math.sqrt(math.pow((goal_pose_x - self.pose_x), 2) + math.pow((goal_pose_y - self.pose_y), 2))
            error_yaw = ang - self.yaw_deg
            out_yaw = PID_Yaw.set_current_error(error_yaw)
            out_distance = PID_Distance.set_current_error(distance)
            self.action(out_distance, out_yaw)
            t1 = perf_counter()
            if (t1-t0)>120 and (goal_pose_x != 0.0 and goal_pose_y !=0.0):
                self.error_fl=True
                break
        if(goal_pose_x == 0.0 and goal_pose_y == 0.0):
            print("Home position")
        elif(self.error_fl):
            print("Navigation error")
        else:
            print("Reach to the desired point")

        self.action(0, 0)
    def action(self, U1, U2):
        twist = Twist()
        twist.linear.x = U1
        twist.angular.z = U2
        self.velocity_publisher.publish(twist)
