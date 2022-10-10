import rospy


class NavigationMaxTimeError(Exception):
    time: int = 100

    def __init__(self, message: str):
        rospy.logerr(message)
