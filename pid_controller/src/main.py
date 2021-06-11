#!/usr/bin/env python
import rospy
from test import TEST
from pyswarm import pso


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node('turtle_bot_pid', anonymous=True, disable_signals=True)

    test = TEST("trajectory", "distance", 40, 1, 0.1)
    # type="distance" or "yaw", max_time, belbic= 0 or 1 (on off), distance):

    lb = [0.001, 0.001, 0.001]
    ub = [2, 2, 2]
    # lb = [1.0, 1.0, 0.0]
    # ub = [2, 2, 0.5]
    if test.test_main_type == "trajectory":
        xopt, fopt = pso(test.test_trajectory, lb, ub, minfunc=0.001, minstep=0.001)
    elif test.test_main_type == "points":
        xopt, fopt = pso(test.test_points, lb, ub, minfunc=0.001, minstep=0.001)
    print("Xopt :", xopt)
    print("Fopt :", fopt)

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
