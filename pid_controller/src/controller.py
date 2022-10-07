# import json
import logging
from datetime import datetime
from math import pow, sqrt
from os import name, system
from time import perf_counter

import rospy
from std_srvs.srv import Empty

from errors import NavigationMaxTimeError
from turtlebotnode import TurtleBotNode

reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)


def calc_distance(
    p_start: tuple[float, float], p_end: tuple[float, float], acc: int = 4
) -> float:
    return round(
        sqrt(pow((p_start[0] - p_end[0]), 2) + pow((p_start[1] - p_end[1]), 2)), acc
    )


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
        self.logger = logging.getLogger("Controller")
        self.turtlebot = TurtleBotNode(max_time)
        self.turtlebot.accuracy = dist_acc
        self.belbic = belbic
        self.tests_results: dict = {"positive": 0, "negative": 0}
        self.best_results: dict = {"error": 100.0, "time": 3600, "pid": [0, 0, 0]}

        self.points: list[float, float] = None
        self.file_name: str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        reset_simulation()
        system("cls" if name == "nt" else "clear")

    def tuning_unity(self, pid: list[float, float, float]):
        """
        One point test without belblic controller

        Args:
            pid (list[float, float, float]): [description]

        Returns:
            float: [description]
        """
        pid = [round(i, 4) for i in pid]

        destination = None  # Cel pobierany z generatora który odczytuje dane z pliku
        distance = calc_distance(destination, self.turtlebot.pose)

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

        # Logging information
        self.logger.info(
            f"Pozycja : {self.turtlebot.pose}"
            f"Czas: {time_of_moving} Błąd: {error_of_moving} "
            f"Dystans do celu: {distance} "
        )

        # Check if the error is the best
        if error_of_moving < self.best_results.get("error"):
            self.best_results["time"] = time_of_moving
            self.best_results["error"] = error_of_moving

            self.logger.info(f"Nowy najlepszy wynik: {pid} Error : {error_of_moving}")
            with open(self.file_name, "a") as fp:
                fp.write(
                    f"\n{error_of_moving}|{time_of_moving}"
                    f"|{distance}|{pid[0]}|{pid[1]}|{pid[2]}"
                )
        self.tests_results["positive"] += 1
        return error_of_moving

    def tuning_complex(self, pid: list[float, float, float]):

        pid = [round(i, 4) for i in pid]

        total_distance: list = None
        error: list = None
        total_time: int = 0

        for num, destination in enumerate(self.points()):  # self.points or points_3

            distance = calc_distance(destination, self.turtlebot.pose)
            self.logger.debug(f"{num} Dest: {destination} dist: {distance}")

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
            self.logger.info(
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
