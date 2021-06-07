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
    ub = [1, 1, 1]
    xopt, fopt = pso(test.test_trajectory, lb, ub, minfunc=0.001, minstep=0.001)
    print("Xopt :", xopt)
    print("Fopt :", fopt)

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
