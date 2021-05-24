from turtlebotnode import TurtleBotNode
from points import points
from time import perf_counter


class TEST:
    start_p = 0.05
    start_i = 0.00
    start_d = 0.01

# Paramtery do zwracania do atuotunera PID
    k_p = 0
    k_i = 0
    k_d = 0
    mean_error=0
    time=0

    def __init__(self):
        self.turtlebot = TurtleBotNode()

    def test_points(self,k):
        result = []
        error = []
        self.time = 0
        self.k_p = k[0]
        self.k_i = k[1]
        self.k_d = k[2]
        result.append(("\nNastawy: ", k[0], k[1], k[2],0,0))
        print("Nastawy: ", k[0], k[1], k[2],0,0)
        for i in points:
            print("Destination :", i[0], " , ", i[1])
            t0 = perf_counter()
            self.turtlebot.calculations(i[0], i[1], k[0], k[1], k[2])
            t1 = perf_counter()
            tmp_time = t1-t0
            error.append(((i[0]-self.turtlebot.pose_x) + (i[1]-self.turtlebot.pose_y)) * tmp_time)
            tmp_time=round(tmp_time,2)
            print("Pose :", self.turtlebot.pose_x, " , ", self.turtlebot.pose_y, " Time: ", tmp_time, " Error: ", round(error[-1],4))
            result.append((i[0], i[1], self.turtlebot.pose_x, self.turtlebot.pose_y, tmp_time,round(error[-1],4)))
            self.time += tmp_time
            if self.turtlebot.error_fl == True :
                    break
        if self.turtlebot.error_fl == True :
                self.go_home()
                return 10000
        with open('/home/jakub/ROS_Projects/Turtle_bot_pid/output/results.txt', 'a') as fp:
            fp.write('\n'.join('{}|{}|{}|{}|{}|{}'.format(x[0], x[1], x[2], x[3], x[4], x[5]) for x in result))
        self.mean_error = sum(error) / len(error)
        string_tmp = '\n'+str(round(self.mean_error,4))+"|"+str(round(self.time,2))+"|"+str(k[0])+"|"+str(k[1])+"|"+str(k[2])
        with open('/home/jakub/ROS_Projects/Turtle_bot_pid/output/error.txt', 'a') as fp:
            fp.write(string_tmp)
        self.go_home()
        return self.mean_error

    def go_home(self):
        self.turtlebot.calculations(0.0, 0.0, self.start_p, self.start_i, self.start_d)
        print("End pose :", self.turtlebot.pose_x, " , ", self.turtlebot.pose_y)
        return True
