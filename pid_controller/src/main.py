#!/usr/bin/env python
import rospy
from test import TEST
from pyswarm import pso


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node('turtle_bot_pid', anonymous=True, disable_signals=True)
    test = TEST()

    lb = [0.001, 0.001, 0.001]
    ub = [0.1, 0.1, 0.1]
    test.turtlebot.go_home_pos()
    xopt, fopt = pso(test.test_points, lb, ub)
    print("Xopt :", xopt)
    print("Fopt :", fopt)

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
