FROM osrf/ros:jazzy-desktop

RUN apt update

RUN apt install -y python3-pip git
# RUN rosdep update --rosdistro jazzy

# RUN pip install colcon-common-extensions
# RUN pip install -U numpy

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
