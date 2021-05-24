#!/usr/bin/env python
import rospy
from test import TEST
from pyswarm import pso

def shutdown_callback():
    print("Shutting down the node")


if __name__ == "__main__":
    rospy.init_node('turtle_bot_pid', anonymous=True, disable_signals=True )
    with open('/home/jakub/ROS_Projects/Turtle_bot_pid/output/results.txt', 'w') as fp:
        fp.write('Dest_x|Dest_y|Pos_x|Pos_y|delta_t|error')
    test = TEST()
    test_pid = 0.05, 0.00, 0.01

    # while(it<iterations):
    #         test.go_home()
    #         test.test_points(0.05, 0.00, 0.01)
    # test.go_home()
    # test.test_points(0.05, 0.00, 0.01)
    # test.go_home()
    lb = [0.03,0.01,0.01]
    ub = [0.07,0.05,0.05]
    xopt, fopt = pso(test.test_points, lb, ub)
    print("Xopt :", xopt)
    print("Fopt :", fopt)

    rospy.on_shutdown(shutdown_callback)
    rospy.signal_shutdown("End")
    rospy.spin()
