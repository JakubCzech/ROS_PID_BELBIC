from turtlebotnode import TurtleBotNode
from points import points_1, points_3
from time import perf_counter
import math

class TEST:

# Paramtery do zwracania do atuotunera PID
    destination = [0,0]
    pid = [0,0,0]
    best_error = 1.0
    iterator = 0

    def __init__(self,type="yaw",max_time=90):
        self.turtlebot = TurtleBotNode(max_time)
        self.type = type # yaw or distance

# Inicjalizacja zmiennych dla testu punktów, odczytanie kolejnego punktu w trasie
    def init_test_points(self):
        self.test_type = "points"
        self.destination[0] = points_1[self.iterator][0]
        self.destination[1] = points_1[self.iterator][1]
        self.iterator += 1

# Test polegający na dobieraniu nastaw dla każdego kolejnego punktu
    def test_points(self, pid):
        self.init_test_points()
        print(self.iterator,".Nastawy: ", pid[0], pid[1], pid[2], " Cel :", self.destination[0], " , ", self.destination[1])
        t0 = perf_counter()
        self.turtlebot.calculations(self.destination,pid,self.type)
        t1 = perf_counter()
        tmp_time = round(t1-t0, 2)
        distance = round(math.sqrt(math.pow((self.destination[0] - self.turtlebot.pose[0]), 2) + math.pow((self.destination[1] - self.turtlebot.pose[1]), 2)), 4)
        tmp_error = round(distance * tmp_time, 4)
        if self.turtlebot.error_fl:
            self.turtlebot.go_point_pos(self.destination)
            return 10000
        print("Pose :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Time: ", tmp_time, " Error: ", tmp_error)
        if tmp_error < self.best_error:
            self.best_error = tmp_error
            self.pid = pid
            print("Update best: ", pid[0], pid[1], pid[2], " Error :", tmp_error)
        self.string_tmp = '\n'+str(self.iterator)+".|"+str(tmp_error)+"|"+str(tmp_time)+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
        self.save_error()
        return tmp_error

# Inicjalizacja zmiennych dla testu trajektorii
    def init_test_trajectory(self, pid):
        self.test_type = "trajectory"
        self.result = []
        self.error = []
        self.time = 0
        self.result.append(("\nNastawy: ", pid[0], pid[1], pid[2], 0, 0))
        print("Nastawy: ", pid[0], pid[1], pid[2])
        if(self.turtlebot.go_home_pos()):
            return True

#  Test dla trajektorii punktów
    def test_trajectory(self, pid):
        self.init_test_trajectory(pid)
        for i in points_3:
            self.destination[0] = i[0]
            self.destination[1] = i[1]
            print("Destination :", self.destination[0], " , ", self.destination[1])
            t0 = perf_counter()
            self.turtlebot.calculations(self.destination, pid, self.type)
            t1 = perf_counter()
            tmp_time = t1-t0
            distance = math.sqrt(math.pow((self.destination[0] - self.turtlebot.pose[0]), 2) + math.pow((self.destination[1] - self.turtlebot.pose[1]), 2))
            self.error.append(distance * tmp_time)
            tmp_time = round(tmp_time, 2)
            print("Pose :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Time: ", tmp_time, " Error: ", round(self.error[-1], 4))
            self.result.append((self.destination[0], self.destination[1], self.turtlebot.pose[0], self.turtlebot.pose[1], tmp_time,round(self.error[-1],4)))
            self.time += tmp_time
            if self.turtlebot.error_fl:
                return 100
        self.save_result()
        self.mean_error = sum(self.error) / len(self.error)
        self.string_tmp = '\n'+str(round(self.mean_error, 4))+"|"+str(round(self.time,2))+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
        self.save_error()
        self.turtlebot.go_home_pos()
        return self.mean_error

# Zapisanie wyników
    def save_error(self):
        file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/error_"+self.test_type+"_"+self.type+".txt"
        with open(file_name, 'a') as fp:
            fp.write(self.string_tmp)

# Zapisanie wyników dla poszczegolnych kroków w teście trajektorii
    def save_result(self):
        file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/results_"+ self.type + ".txt"
        with open(file_name, 'a') as fp:
            fp.write('\n'.join('{}|{}|{}|{}|{}|{}'.format(x[0], x[1], x[2], x[3], x[4], x[5]) for x in self.result))
