#!/usr/bin/env python3

import rospy
from pyswarm import pso

from controller import Controller


def shutdown_callback():
    rospy.loginfo("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node(
        "tb_pid", anonymous=True, disable_signals=True, log_level=rospy.INFO
    )

    lb = [0, 0, 0]
    ub = [2, 2, 2]

    controller = Controller(max_time=120, dist_acc=0.1)
    xopt, fopt = pso(controller.tuning_unity, lb, ub, minfunc=0.1, minstep=0.1)
    rospy.loginfo(f"Xopt : {xopt}")
    rospy.loginfo(f"Fopt : {fopt}")

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
