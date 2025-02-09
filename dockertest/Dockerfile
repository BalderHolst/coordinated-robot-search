FROM ros:jazzy-perception-noble

# Install Gazebo
RUN sudo apt update && \
  sudo apt upgrade -y && \
  sudo apt install -y ros-jazzy-turtlesim && \
  sudo apt install -y ros-jazzy-desktop

# Set environment variables for GUI
ENV DISPLAY=docker.for.mac.host.internal:0
VOLUME ["/tmp/.X11-unix:/tmp/.X11-unix"]
