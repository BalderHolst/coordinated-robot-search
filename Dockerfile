FROM osrf/ros:jazzy-desktop

RUN apt update

RUN apt install -y git

RUN apt install -y ros-jazzy-turtlesim

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

VOLUME ["/tmp/.X11-unix:/tmp/.X11-unix"]
