#!/usr/bin/env python
import rospy
from test import TEST
from pyswarm import pso


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node('belbic_pid', anonymous=True, disable_signals=True )
    test = TEST()
    lb = [0.001, 0.001, 0.001]
    ub = [0.1, 0.1, 0.1]
    xopt, fopt = pso(test.test_trajectory, lb, ub)
    print("Xopt :", xopt)
    print("Fopt :", fopt)
    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
# 0.02778856958089164 0.06277435694497437 0.07051942675312053
