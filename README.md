
## General info
This project can show you how Particle swarm optimization works with simulated turtlebot 3. In addition to the classic PID controller, a modified controller is available with the addition of a Belbic biologically-inspired driver.
	
## Technologies
Project is created with:
* Python3
* Ros Noetic
* pyswarm
	
## Setup

### Without Docker

Clone this repository to your src dir and build it (catkin_make or catkin build)

### With Docker

Clone this repository (or download files: setup.bash and Dockerfile to one directory) and install it by executing setup.bash, this file has create new Docker Image with builded pid_controller package and start new terminal in ros workspace. You can start testing by typing roslaunch pid_controller pid_controller.launch. You can modify tuning params by changing main.py and controller.py files.



