import rospy
from math import pow, sqrt
from turtlebotnode import TurtleBotNode
from time import perf_counter
from numpy import arange
from std_srvs.srv import Empty
from os import system, name

reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)


class TEST:
    # Paramtery do zwracania do atuotunera PID
    destination = [0, 0]
    pid = [0, 0, 0]
    points = []
    best_error = 100.0
    best_time = 3600
    iterator = 0
    ok = 0
    not_ok = 0
    belbic = 0
    test_main_type = ""

    def __init__(self, type_main="trajectory", type="distance", max_time=20, belbic=0, distance=0.01):
        reset_simulation()
        system('cls' if name == 'nt' else 'clear')
        self.test_main_type = type_main
        self.type = type
        self.belbic = belbic
        self.turtlebot = TurtleBotNode(max_time)
        self.turtlebot.accuracy = distance
        self.make_points()

    def test(self, pid):
        if(self.test_main_type == "points" and self.belbic == 0):
            self.init_test_points(pid)
            self.print_header_points()
            # Przemieszczenie robota
            t0 = perf_counter()
            self.turtlebot.calculations(self.destination, pid, self.type)
            t1 = perf_counter()
            distance = self.distance()
            # Sprawdzenie i obsługa niepowodzenia
            if self.turtlebot.error_fl:
                print("Niepowodzenie, pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Dystans do celu:", distance)
                self.turtlebot.go_point_pos(self.destination)
                self.not_ok += 1
                return 100+distance*10
            # Obliczenie czasu i dystansu
            tmp_time = round(t1-t0, 2)
            tmp_error = round(distance * tmp_time**2, 4)
            # Wypisanie informacji
            print("Pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Czas: ", tmp_time, " Błąd: ", tmp_error, " Dystans do celu:", distance)
            # Sprawdzenie czy wynik jest lepszy od dotychczasowego najlepszego
            if tmp_error < self.best_error:
                self.best_time = tmp_time
                self.best_error = tmp_error
                print("Nowy najlepszy wynik: ", pid[0], pid[1], pid[2], " Error :", tmp_error)
                self.string_tmp = '\n'+str(tmp_error)+"|"+str(tmp_time)+"|"+str(distance)+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
                self.save()
            # Aktualizacja licznika i zwracanie błędu
            self.ok += 1
            return tmp_error

        elif(self.test_main_type == "trajectory" and self.belbic == 0):
            self.init_test_trajectory(pid)
            it = 0
            distance = []
            time = 0
            error = []
            for i in self.points:  # self.points or points_3
                it += 1
                # Wyznaczenie celu
                self.destination = i
                print(it, "Cel :", self.destination[0], " , ", self.destination[1])
                # Przemieszczenie robota
                t0 = perf_counter()
                self.turtlebot.calculations(self.destination, pid, self.type)
                t1 = perf_counter()
                # Obliczenie czasu przemieszczenia
                tmp_time = round(t1-t0, 4)
                time += tmp_time
                # Obliczenie dystansu
                tmp_distance = self.distance()
                # Sprawdzenie i obsługa niepowodzenia oraz czy trajektoria nie trwa zbyt długo
                if self.turtlebot.error_fl or time > self.best_time:
                    self.not_ok += 1
                    return 1000-(100*it)+tmp_distance*10
                # Dopisanie zmiennych z aktualnego punktu do statystyk
                error.append(round(tmp_distance * tmp_time**2, 4))
                distance.append(tmp_distance)
                # Wypisanie informacji
                print("Pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Czas: ", tmp_time, " Błąd: ", round(error[-1], 4), " Dystans do celu:", tmp_distance)
            # Dotarto do celu z nowym najlepszym czasem
            mean_error = round(sum(error) / len(error), 4)
            mean_distance = round(sum(distance) / len(distance), 4)
            self.best_time = time
            self.best_error = mean_error
            # Wygenerowanie lini do pliku i zapisanie
            self.string_tmp = '\n'+str(mean_error)+"|"+str(time)+"|"+str(mean_distance)+"|"+str(pid[0])+"|"+str(pid[1])+"|"+str(pid[2])
            self.save()
            # Aktualizacja licznik udanych trajektorii
            self.ok += 1
            # Zwrócenie błędu
            return self.mean_error

        elif(self.test_main_type == "trajectory" and self.belbic == 1):
            self.print_header_belbic()
            it = 0
            distance = []
            error = []
            time = 0
            for i in self.points:  # self.points or points_3
                it += 1
                self.destination = i
                print(it, "Cel :", self.destination[0], " , ", self.destination[1])
                # Przemieszczenie robota
                t0 = perf_counter()
                self.turtlebot.calculations_belbic(self.destination, pid, self.type)
                t1 = perf_counter()
                # Obliczenie czasu i dystansu
                tmp_time = round(t1-t0, 4)
                tmp_distance = self.distance()
                time += tmp_time
                # Sprawdzenie i obsługa niepowodzenia oraz czy trajektoria nie trwa zbyt długo
                if self.turtlebot.error_fl or time > self.best_time:
                    self.not_ok += 1
                    return 1000-(100*it)+tmp_distance*10

                error.append(round(tmp_distance * tmp_time**2, 4))
                print("Pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Czas: ", tmp_time, " Błąd: ", round(error[-1], 4), " Dystans do celu:", tmp_distance)
                # Dodanie wartości tymczasowych
                distance.append(tmp_distance)
            # Policzenie średniego dystansu i błędu
            mean_distance = round(sum(distance) / len(distance), 4)
            mean_error = round(sum(error) / len(error), 4)
            # Przypisanie nowego najlepszego czasu
            self.best_time = time
            self.best_error = mean_error
            # Zapisanie danych do pliku
            self.string_tmp = '\n'+str(mean_error)+"|"+str(time)+"|"+str(mean_distance)
            self.save()
            # Aktualizacja licznik udanych trajektorii
            self.ok += 1
            # Zwrócenie błędu
            return mean_error

        elif(self.test_main_type == "points" and self.belbic == 1):
            self.init_test_points(pid)
            self.print_header_belbic()
            # Przemieszczenie robota
            t0 = perf_counter()
            self.turtlebot.calculations_belbic(self.destination, pid, self.type)
            t1 = perf_counter()
            distance = self.distance()
            # Sprawdzenie i obsługa niepowodzenia
            if self.turtlebot.error_fl:
                print("Niepowodzenie, pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Dystans do celu:", distance)
                reset_simulation()
                self.turtlebot.go_point_pos(self.destination)
                self.not_ok += 1
                return 100+distance*10
            # Obliczenie czasu i dystansu
            tmp_time = round(t1-t0, 2)
            tmp_error = round(distance * tmp_time**2, 4)
            # Wypisanie informacji
            print("Pozycja :", self.turtlebot.pose[0], " , ", self.turtlebot.pose[1], " Czas: ", tmp_time, " Błąd: ", tmp_error, " Dystans do celu:", distance)
            # Sprawdzenie czy wynik jest lepszy od dotychczasowego najlepszego
            if tmp_error < self.best_error:
                self.best_error = tmp_error
                self.best_time = tmp_time
                print("Nowy najlepszy wynik: ", pid[0], pid[1], pid[2], " Error :", tmp_error)
                self.string_tmp = '\n'+str(tmp_error)+"|"+str(tmp_time)+"|"+str(distance)
                self.save()
            # Aktualizacja licznika i zwracanie błędu
            self.ok += 1
            return tmp_error

    # Inicjalizacja zmiennych dla testu punktów, odczytanie kolejnego punktu w trasie
    def init_test_points(self, pid):
        pid[0] = round(pid[0], 4)
        pid[1] = round(pid[1], 4)
        pid[2] = round(pid[2], 4)
        self.pid = pid
        self.destination[0] = self.points[self.iterator][0]
        self.destination[1] = self.points[self.iterator][1]
        self.iterator += 1
        if (self.iterator == len(self.points)):
            self.iterator = 0
            system('cls' if name == 'nt' else 'clear')

    # Inicjalizacja zmiennych dla testu trajektorii
    def init_test_trajectory(self, pid):
        pid[0] = round(pid[0], 4)
        pid[1] = round(pid[1], 4)
        pid[2] = round(pid[2], 4)
        self.pid = pid
        system('cls' if name == 'nt' else 'clear')
        reset_simulation()
        self.iterator += 1
        self.print_header_trajectory()

    # Zapisanie wyników
    def save(self):
        if(self.belbic):
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/belbic/error_"+self.test_main_type+"_"+self.type+"_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        else:
            file_name = "/home/jakub/ROS_Projects/Turtle_bot_pid/output/pid/error_"+self.test_main_type+"_"+self.type+"_"+str(self.turtlebot.accuracy)+"_"+str(self.turtlebot.max_time)+".txt"
        with open(file_name, 'a') as fp:
            fp.write(self.string_tmp)

    def make_points(self):
        for i in arange(1.0, 11.0, 1.0):
            if(i % 4 == 1):
                self.points.append(((float(i)/2)*2, 2))
            elif(i % 4 == 2):
                self.points.append(((float(i)/2)*2, 0))
            elif(i % 4 == 3):
                self.points.append(((float(i)/2)*2, -2))
            else:
                self.points.append(((float(i)/2)*2, 0))
        if(self.test_main_type == "points"):
            for i in arange(9.0, -1.0, -1.0):
                if(i % 4 == 1):
                    self.points.append(((float(i)/2)*2, -2))
                elif(i % 4 == 2):
                    self.points.append(((float(i)/2)*2, 0))
                elif(i % 4 == 3):
                    self.points.append(((float(i)/2)*2, 2))
                else:
                    self.points.append(((float(i)/2)*2, 0))

    def distance(self):
        return round(sqrt(pow((self.destination[0] - self.turtlebot.pose[0]), 2) + pow((self.destination[1] - self.turtlebot.pose[1]), 2)), 4)

    def print_header_points(self):
        if self.ok+self.not_ok > 0:
            print("")
            print("Prób udanych:", self.ok, "Prób nieudanych:", self.not_ok, "Procent poprawnych:", round(100*(self.ok/(self.ok+self.not_ok)), 2), "% Najlepszy czas:", self.best_time)
        print(self.ok+self.not_ok+1, "Nastawy: ", self.pid[0], self.pid[1], self.pid[2], " Cel :", self.destination[0], " , ", self.destination[1])

    def print_header_trajectory(self):
        if self.ok+self.not_ok > 0:
            print("Prób udanych:", self.ok, "Prób nieudanych:", self.not_ok, "Procent poprawnych:", round(100*(self.ok/(self.ok+self.not_ok)), 2), "% Najlepszy czas:", self.best_time)
            print("Próba: ", self.iterator, "Nastawy: ", self.pid[0], self.pid[1], self.pid[2])
            print("")

    def print_header_belbic(self):
        if(self.test_main_type == "trajectory"):
            system('cls' if name == 'nt' else 'clear')
        if self.ok+self.not_ok > 0:
            print("")
            print("Prób udanych:", self.ok, "Prób nieudanych:", self.not_ok, "Procent poprawnych:", round(100*(self.ok/(self.ok+self.not_ok)), 2), "% Najlepszy czas:", self.best_time)
        print(self.ok+self.not_ok+1, " Cel :", self.destination[0], " , ", self.destination[1])
