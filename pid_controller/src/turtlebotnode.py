#!/usr/bin/env python

import rospy
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry
from pid_controller import PID_CONTROLLER
from geometry_msgs.msg import Twist
from time import perf_counter,sleep


class TurtleBotNode:

    def __init__(self,max_time):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rospy.Subscriber("odom", Odometry, self.callback_update_position)
        self.pose = [1,0]
        self.yaw_deg = 0
        self.error_fl = 0
        self.max_time=max_time
        self.rate = rospy.Rate(10)
        self.action(0, 0)
        self.go_home_pos()

    def callback_update_position(self, data):
        self.pose[0]= round(data.pose.pose.position.x, 4)
        self.pose[1]= round(data.pose.pose.position.y, 4)
        quaternion = data.pose.pose.orientation

        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.yaw_deg = round(math.degrees(yaw), 4)

    def calculations(self,destination,pid,type): #destination(x,y),pid(p,i,d),test_type(yaw or distance)
        goal_pose = destination
        accuracy = 0.1

        if(type=="yaw"):
            PID_Yaw = PID_CONTROLLER(pid[0], pid[1], pid[2], 1)
            PID_Distance = PID_CONTROLLER(0.0655145404317901, 0.09993384022612885, 0.028555576446369184, 1)
        elif(type=="distance"):
            PID_Distance= PID_CONTROLLER(pid[0], pid[1], pid[2], 1)
            PID_Yaw = PID_CONTROLLER(0.05550086361364029, 0.04983812908133718, 0.04405278167220987, 1)

        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        t0 = perf_counter()
        self.error_fl = False

        while distance > accuracy:
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
        if(self.error_fl):
            print("Navigation error")
            self.action(0, 0)
        self.action(0, 0)


    def go_home_pos(self):
        self.action(0, 0)
        goal_pose = (0.0,0.0)
        accuracy = 0.001
        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        PID_Yaw = PID_CONTROLLER(0.05, 0.00, 0.01, 0.3)
        PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.5)
        while distance > accuracy:
            psi = math.atan2(goal_pose[1] - self.pose[1], goal_pose[0] - self.pose[0])
            ang = math.degrees(psi)
            distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
            error_yaw = ang - self.yaw_deg
            out_yaw = PID_Yaw.set_current_error(error_yaw)
            out_distance = PID_Distance.set_current_error(distance)
            self.action(out_distance, out_yaw)
        self.action(0, 0)
        return True

    def go_point_pos(self, goal_pose):
        accuracy = 0.001
        distance = math.sqrt(math.pow((goal_pose[0] - self.pose[0]), 2) + math.pow((goal_pose[1] - self.pose[1]), 2))
        PID_Yaw = PID_CONTROLLER(0.05, 0.00, 0.01, 0.3)
        PID_Distance = PID_CONTROLLER(0.001, 0.1, 1.6, 0.5)
        while distance > accuracy:
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
