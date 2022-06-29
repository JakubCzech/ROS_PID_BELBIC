# ROS_PID_BELBIC
Węzeł ROS napisany w python służący do testowania algorytmów dobierającyh nastawy PID w celu poruszania TurtleBotem
```
FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND noninteractive 

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN apt-get update && apt-get install -y \
terminator \
git \
python3-pip

RUN mkdir -p /root/miapr/src
RUN cd /root/miapr/src && git clone https://github.com/JakubCzech/ROS_PID_BELBIC.git && $ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN cp -r /root/miapr/ROS_PID_BELBIC/pid_controller/ /root/miapr/src/
RUN cd /root/miapr && catkin_make
RUN source source /opt/ros/noetic/setup.bash && source /root/miapr/devel/setup.bash
RUN pip install pyswarm

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["terminator"]
```
