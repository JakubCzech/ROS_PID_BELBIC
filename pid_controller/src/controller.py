from datetime import datetime
from os import name, system
from typing import Tuple

import rospy
from std_srvs.srv import Empty

from errors import NavigationMaxTimeError
from turtlebotnode import TurtleBotNode


def reset_simulation():
    try:
        rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}", logger_name="Controller")


def calc_error(distance: float, time_of_moving: float, acc: int = 4) -> float:
    """Function to calculate error

    Args:
        distance (float): distance to destination
        time_of_moving (float): time of moving

    Returns:
        _type_: float - error
    """
    return round(distance * time_of_moving ** 2, acc)


class Controller:
    """
    Main controller for TurtlebotSimulator

    """

    def __init__(
        self, *, max_time: int = 20, dist_acc: float = 0.01, belbic: bool = False
    ):

        self.turtlebot = TurtleBotNode(max_time)
        self.turtlebot.accuracy = dist_acc
        self.belbic = belbic

        self.tests_results: dict = {"positive": 0, "negative": 0}
        self.best_results: dict = {"error": 100.0, "time": 3600, "pid": [0, 0, 0]}
        self.file_name: str = (
            f"/root/{datetime.now().strftime('%Y_%m_%d-%H_%M_%S')}.txt"
        )

        with open(self.file_name, "a") as fp:
            fp.write(f"Error_of_moving|Time_of_moving|Distance|P|I|D")

        system("cls" if name == "nt" else "clear")
        reset_simulation()

    def tuning_unity(
        self,
        pid: Tuple[float, float, float],
        destination: Tuple[float, float] = (2.5, 2.5),
    ):
        """
        One point test without belblic controller

        Args:
            pid (Union[float, float, float]): [description]

        Returns:
            float: [description]
        """
        system("cls" if name == "nt" else "clear")
        reset_simulation()

        rospy.loginfo(
            f"Start new unit test, PID: {pid}, time: {rospy.get_time()}",
            logger_name="Controller",
        )

        # Start of robot navigation to the point
        try:
            if self.belbic:
                distance, time_of_moving = self.turtlebot.calculations_belbic(
                    destination, pid
                )
            else:
                distance, time_of_moving = self.turtlebot.calculations(destination, pid)
        except NavigationMaxTimeError:
            self.tests_results["negative"] += 1
            return NavigationMaxTimeError.time

        # Calculating error
        error_of_moving = calc_error(distance, time_of_moving)

        # Logging information
        rospy.loginfo(
            f"Pozycja : {self.turtlebot.pose}"
            f"Czas: {time_of_moving} Błąd: {error_of_moving} "
            f"Dystans do celu: {distance} ",
            logger_name="Controller",
        )

        # Check if the error is the best
        if error_of_moving < self.best_results.get("error"):
            self.best_results["time"] = time_of_moving
            self.best_results["error"] = error_of_moving

            rospy.loginfo(
                f"New best controller: {pid} Error : {error_of_moving}",
                logger_name="Controller",
            )
            with open(self.file_name, "a") as fp:
                fp.write(
                    f"\n{error_of_moving}|{time_of_moving}"
                    f"|{distance}|{pid[0]}|{pid[1]}|{pid[2]}"
                )
        self.tests_results["positive"] += 1

        reset_simulation()

        return error_of_moving

    def tuning_complex(self, pid: Tuple[float, float, float]):

        pid = [round(i, 4) for i in pid]

        distance_hist = list()
        error = list()
        total_time: int = 0

        for destination in self.points():  # self.points or points_3

            # Start of robot navigation to the point
            try:
                if self.belbic:
                    distance, time_of_moving = self.turtlebot.calculations_belbic(
                        destination, pid
                    )
                else:
                    distance, time_of_moving = self.turtlebot.calculations(
                        destination, pid
                    )
            except NavigationMaxTimeError:
                self.tests_results["negative"] += 1
                return NavigationMaxTimeError.time

            # Calculating error
            error_of_moving = calc_error(distance, time_of_moving)
            total_time += time_of_moving
            error.append(error_of_moving)
            distance_hist.append(distance)

            # Logging information
            rospy.logdebug(
                f"Pozycja : {self.turtlebot.pose}"
                f"Czas: {time_of_moving} Błąd: {error_of_moving} "
                f"Dystans do celu: {distance} ",
                logger_name="Controller",
            )
        if self.best_results["error"] > sum(error):
            self.best_results["error"] = sum(error)
            self.best_results["mean_error"] = round(sum(error) / len(error), 4)
            self.best_results["mean_distance"] = round(
                sum(distance_hist) / len(distance_hist), 4
            )
            self.best_results["time"] = total_time

        self.tests_results["positive"] += 1
        return round(sum(error) / len(error), 4)

    def points(self):
        """
        Generator of points

        Yields:
            Tuple[float, float]: [description]
        """
        yield (2.0, 2.0)
        yield (2.0, -2.0)
        yield (-2.0, -2.0)
        yield (-2.0, 2.0)
