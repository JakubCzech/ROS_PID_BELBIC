# import json
from datetime import datetime
from os import name, system
from time import perf_counter
from typing import Tuple, Union

import rospy
from std_srvs.srv import Empty

from errors import NavigationMaxTimeError
from turtlebotnode import TurtleBotNode, calc_distance

reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)


def calc_error(distance: float, time_of_moving: float, acc: int = 4) -> float:
    """Function to calculate error

    Args:
        distance (float): distance to destination
        time_of_moving (float): time of moving

    Returns:
        _type_: float - error
    """
    return round(distance * time_of_moving**2, acc)


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

        self.points: Union[float, float] = None
        self.file_name: str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        try:
            reset_simulation()
        except rospy.ServiceException as e:
            rospy.logerr(e)
        # system("cls" if name == "nt" else "clear")

    def tuning_unity(self, pid: Union[float, float, float]):
        """
        One point test without belblic controller

        Args:
            pid (Union[float, float, float]): [description]

        Returns:
            float: [description]
        """
        pid = [round(i, 4) for i in pid]

        destination = (
            1.5,
            1.5,
        )  # Cel pobierany z generatora który odczytuje dane z pliku

        try:
            reset_simulation()
        except rospy.ServiceException as e:
            rospy.logerr(e)

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
        rospy.logerr(
            f"Pozycja : {self.turtlebot.pose}"
            f"Czas: {time_of_moving} Błąd: {error_of_moving} "
            f"Dystans do celu: {distance} "
        )

        # Check if the error is the best
        if error_of_moving < self.best_results.get("error"):
            self.best_results["time"] = time_of_moving
            self.best_results["error"] = error_of_moving

            rospy.logerr(f"Nowy najlepszy wynik: {pid} Error : {error_of_moving}")
            with open(self.file_name, "a") as fp:
                fp.write(
                    f"\n{error_of_moving}|{time_of_moving}"
                    f"|{distance}|{pid[0]}|{pid[1]}|{pid[2]}"
                )
        self.tests_results["positive"] += 1
        return error_of_moving

    def tuning_complex(self, pid: Union[float, float, float]):

        pid = [round(i, 4) for i in pid]

        total_distance = None
        error = None
        total_time: int = 0

        for num, destination in enumerate(self.points()):  # self.points or points_3

            distance = calc_distance(destination, self.turtlebot.pose)
            rospy.logdebug(f"{num} Dest: {destination} dist: {distance}")

            # Start of robot navigation to the point
            start_time = perf_counter()
            try:
                if self.belbic:
                    self.turtlebot.calculations_belbic(destination, pid, "points")
                else:
                    self.turtlebot.calculations(destination, pid, "points")
            except NavigationMaxTimeError:
                self.tests_results["negative"] += 1
                return NavigationMaxTimeError.time + distance * 10
            end_time = perf_counter()

            # Calculating error
            time_of_moving = round(end_time - start_time, 4)
            error_of_moving = calc_error(distance, time_of_moving)
            total_time += time_of_moving
            error.append(error_of_moving)
            total_distance.append(distance)

            # Logging information
            rospy.loginfo(
                f"Pozycja : {self.turtlebot.pose}"
                f"Czas: {time_of_moving} Błąd: {error_of_moving} "
                f"Dystans do celu: {distance} "
            )
        if self.best_results["error"] > sum(error):
            self.best_results["error"] = sum(error)
            self.best_results["mean_error"] = round(sum(error) / len(error), 4)
            self.best_results["mean_distance"] = round(
                sum(total_distance) / len(total_distance), 4
            )
            self.best_results["time"] = total_time

        self.tests_results["positive"] += 1
        return round(sum(error) / len(error), 4)
