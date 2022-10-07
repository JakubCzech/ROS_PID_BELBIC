#!/usr/bin/env python
import logging

import rospy
from pyswarm import pso

from controller import Controller


def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node("tb_pid", anonymous=True, disable_signals=True)

    lb = [0, 0, 0]
    ub = [2, 2, 2]

    controller = Controller(max_time=60, dist_acc=0.01)
    # type="points" or "trajectory",type="distance" or "yaw", max_time, belbic= 0 or 1 (on off), distance):

    xopt, fopt = pso(controller.tuning_complex, lb, ub, minfunc=0.1, minstep=0.1)
    logging.info(f"Xopt : {xopt}")
    logging.info(f"Fopt : {fopt}")

    controller = Controller(max_time=60, dist_acc=0.01)
    # type="points" or "trajectory",type="distance" or "yaw", max_time, belbic= 0 or 1 (on off), distance):

    xopt, fopt = pso(controller.tuning_unity, lb, ub, minfunc=0.1, minstep=0.1)
    logging.info(f"Xopt : {xopt}")
    logging.info(f"Fopt : {fopt}")

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
