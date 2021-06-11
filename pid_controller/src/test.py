import rospy
import math

from turtlebotnode import TurtleBotNode
from points import points_3

from time import perf_counter
from numpy import arange
from std_srvs.srv import Empty

from os import system, name


reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)


class TEST:
    # Paramtery do zwracania do atuotunera PID
    destination = [0, 0]
    pid = [0, 0, 0]
    best_error = 10.0
    iterator = 0
    points = []
    ok = 0
    not_ok = 0
    best_time = 50
    belbic = 0
    test_type = ""
    test_main_type = ""

    def __init__(self,type_main="trajectory", type="distance", max_time=20, belbic=0, distance=0.01):
        reset_simulation()
        system('cls' if name == 'nt' else 'clear')
        self.test_main_type = type_main
        self.turtlebot = TurtleBotNode(max_time)
        self.type = type
        self.belbic = belbic
        self.turtlebot.accuracy = distance
        for i in arange(1.0, 11.0, 1.0):
            self.points.append((float(i)/2, i % 2))
        for i in arange(9.0, -1.0, -1.0):
            self.points.append((float(i)/2, i % 2))
    # Inicjalizacja zmiennych dla testu punktów, odczytanie kolejnego punktu w trasie
    def init_test_points(self):
        self.test_type = "points"
        self.destination[0] = self.points[self.iterator][0]
        self.destination[1] = self.points[self.iterator][1]
        self.iterator += 1
        if (self.iterator == 20):
            self.iterator = 0
            system('cls' if name == 'nt' else 'clear')

    # Test polegający na dobieraniu nastaw dla każdego kolejnego punktu

    def test_points(self, pid):
        self.init_test_points()
        if self.ok+self.not_ok > 0:
            print("Prób udanych:", self.ok, "Prób nieudanych:", self.not_ok, "Procent poprawnych:", round(100*(self.ok/(self.ok+self.not_ok)), 2), "% Najlepszy wynik:", self.best_error)
        print(self.iterator, "Nastawy: ", pid[0], pid[1], pid[2], " Cel :", self.destination[0], " , ", self.destination[1])
        t0 = perf_counter()
        if(self.belbic):
            self.turtlebot.calculations_belbic(self.destination, pid, self.type)
        else:
            self.turtlebot.calculations(self.destination, pid, self.type)
        t1 = perf_counter()
        tmp_time = round(t1-t0, 2)
        distance = round(math.sqrt(math.pow((self.destination[0] - self.turtlebot.pose[0]), 2) + math.pow((self.destination[1] - self.turtlebot.pose[1]), 2)), 4)
        tmp_error = round(distance * tmp_time**2, 4)
        if self.turtlebot.error_fl:
            reset_simulation()
            self.turtlebot.go_point_pos(self.destination)
            self.not_ok += 1
            return 100+distance*10

        print("Pose :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Time: ", tmp_time, " Error: ", tmp_error)
        if tmp_error < self.best_error:
            self.best_error = tmp_error
            self.pid = pid
            print("Update best: ", pid[0], pid[1], pid[2], " Error :", tmp_error)
        self.string_tmp = '\n'+str(self.iterator)+".|"+str(tmp_error)+"|"+str(tmp_time)+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
        self.save_error()
        self.ok += 1
        return tmp_error

    # Inicjalizacja zmiennych dla testu trajektorii
    def init_test_trajectory(self, pid):
        system('cls' if name == 'nt' else 'clear')
        self.test_type = "trajectory"
        reset_simulation()
        self.result = []
        self.error = []
        self.time = 0
        pid[0] = round(pid[0], 4)
        pid[1] = round(pid[1], 4)
        pid[2] = round(pid[2], 4)
        self.iterator += 1
        if(self.iterator > 1):
            print("Prób udanych:", self.ok, "Prób nieudanych:", self.not_ok, "Procent poprawnych:", round(100*(self.ok/(self.ok+self.not_ok)), 2), "% Najlepszy czas:", self.best_time)
        self.result.append(("\nNastawy: ", pid[0], pid[1], pid[2], 0, 0))
        print("Próba: ", self.iterator, "Nastawy: ", pid[0], pid[1], pid[2])

    #  Test dla trajektorii punktów
    def test_trajectory(self, pid):
        self.init_test_trajectory(pid)
        it = 0
        for i in points_3:
            it += 1
            self.destination[0] = i[0]
            self.destination[1] = i[1]
            print(it, "Cel :", self.destination[0], " , ", self.destination[1])
            t0 = perf_counter()
            if(self.belbic):
                self.turtlebot.calculations_belbic(self.destination, pid, self.type)
            else:
                self.turtlebot.calculations(self.destination, pid, self.type)
            t1 = perf_counter()
            distance = math.sqrt(math.pow((self.destination[0] - self.turtlebot.pose[0]), 2) + math.pow((self.destination[1] - self.turtlebot.pose[1]), 2))
            if self.turtlebot.error_fl:
                self.not_ok += 1
                return 1000-(100*it)+distance*10
            tmp_time = round(t1-t0, 4)
            self.error.append(round(distance * tmp_time**2, 4))
            print("Pose :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Time: ", tmp_time, " Error: ", round(self.error[-1], 4))
            self.result.append((self.destination[0], self.destination[1], self.turtlebot.pose[0], self.turtlebot.pose[1], tmp_time, self.error[-1]))
            self.time += tmp_time
            if(self.time > self.best_time):
                self.not_ok += 1
                return 1000-(100*it)+distance*10
        if(self.time < self.best_time):
            self.best_time = self.time
        self.save_result()
        self.ok += 1
        self.mean_error = sum(self.error) / len(self.error)
        self.string_tmp = '\n'+str(round(self.mean_error, 4))+"|"+str(round(self.time, 2))+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
        self.save_error()
        return self.mean_error

    # Zapisanie wyników
    def save_error(self):
        if(self.belbic):
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/belbic/error_"+self.test_type+"_"+self.type+"_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        else:
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/pid/error_"+self.test_type+"_"+self.type+"_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        with open(file_name, 'a') as fp:
            fp.write(self.string_tmp)

    # Zapisanie wyników dla poszczegolnych kroków w teście trajektorii
    def save_result(self):
        if(self.belbic):
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/belbic/results_" + self.type +"_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        else:
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/pid/results_" + self.type + "_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        with open(file_name, 'a') as fp:
            fp.write('\n'.join('{}|{}|{}|{}|{}|{}'.format(x[0], x[1], x[2], x[3], x[4], x[5]) for x in self.result))
