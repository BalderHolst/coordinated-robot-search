FROM osrf/ros:jazzy-desktop


RUN apt update

RUN apt install -y git

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN rustc --version && cargo --version

RUN apt update
RUN apt -y install curl lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update
RUN apt install -y gz-harmonic

RUN apt install -y ros-jazzy-nav2*
RUN apt install -y ros-jazzy-turtlesim
RUN apt install -y libopencv-dev clang libclang-dev # For opencv-rust

# Install libc
RUN apt-get update && apt-get install -y curl build-essential gcc libc6-dev
env C_INCLUDE_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/include

# Set up ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Terminal multiplexer for convenience
RUN apt install -y tmux
