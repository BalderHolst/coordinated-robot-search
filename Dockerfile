FROM osrf/ros:jazzy-desktop


RUN apt update

RUN apt install -y git


RUN apt update
RUN apt -y install curl lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update
RUN apt install -y gz-harmonic

RUN apt install -y ros-jazzy-nav2*
RUN apt install -y ros-jazzy-turtlesim

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# terminal multiplexer for convenience
RUN apt install -y tmux
