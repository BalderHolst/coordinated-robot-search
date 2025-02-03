FROM osrf/ros:foxy-desktop

RUN apt update

RUN apt install -y python3-pip git
# RUN rosdep update --rosdistro foxy

RUN pip install colcon-common-extensions
RUN pip install -U numpy

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
