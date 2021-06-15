#!/usr/bin/env python
import rospy
from test import TEST
from pyswarm import pso


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node('turtle_bot_pid', anonymous=True, disable_signals=True)

    lb = [0, 0, 0]
    ub = [2, 2, 2]

    test = TEST("points", "distance", 50, 1, 0.05)
    # type="points" or "trajectory",type="distance" or "yaw", max_time, belbic= 0 or 1 (on off), distance):

    xopt, fopt = pso(test.test, lb, ub, minfunc=0.1, minstep=0.1)
    print("Xopt :", xopt)
    print("Fopt :", fopt)

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
