# ROS_PID_BELBIC
Węzeł ROS napisany w python służący do testowania algorytmów dobierającyh nastawy PID w celu poruszania TurtleBotem
```
FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND noninteractive 

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN apt-get update && apt-get install -y terminator git python3-pip \
ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 \
ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs \
ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf \ 
ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers black \
&& apt-get clean -y && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/miapr/src
RUN cd /root/miapr/src && git clone https://github.com/JakubCzech/ROS_PID_BELBIC.git && git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN cp -r /root/miapr/src/ROS_PID_BELBIC/pid_controller /root/miapr/src/
RUN rm -rf /root/miapr/src/ROS_PID_BELBIC
RUN chmod +x /root/miapr/src/pid_controller/src/*
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /root/miapr ; catkin_make ; source devel/setup.bash'
RUN pip install pyswarm
RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["terminator"]
```
