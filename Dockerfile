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
RUN pip install pyswarm simple-pid

RUN mkdir -p /root/ros_pid_belbic/src

WORKDIR /root/ros_pid_belbic/src

RUN git clone https://github.com/JakubCzech/ROS_PID_BELBIC.git 
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN cp -r ROS_PID_BELBIC/pid_controller .
RUN rm -rf ROS_PID_BELBIC
RUN chmod +x pid_controller/src/*

WORKDIR /root/ros_pid_belbic

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make ; source devel/setup.bash'
ENV TURTLEBOT3_MODEL=burger
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["terminator"]