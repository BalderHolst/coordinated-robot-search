# Coordinated Robot Search

Bachelor Project 2025

- [Thesis Report](https://balderholst.github.io/coordinated-robot-search/report.pdf)
- [botbrain documentation](https://balderholst.github.io/coordinated-robot-search/docs/botbrain/botbrain)
---

## [`botbrain`](./botbrain)

Rust library crate containing the core logic of the project including the search algorithms.

Documentation can be found [here](https://balderholst.github.io/coordinated-robot-search/docs/botbrain/botbrain).

## [`simple_sim`](./simple_sim)

A simple simulator for running behaviors defined in [`botbrain`](#botbrain).

Start the simulation with:

```bash
cd simple_sim
cargo run --release -- run ../worlds/bitmap/depot/depot.yaml search:pathing
```

When the simulator opens, click `Spawn Robots` and click on the map to spawn robots. The speed of the simulation can be adjusted by dragging the slider in the bottom left. Information in the `Debug Soup` can be visualized by clicking on the robot and choosing a `Debug Item` in the right panel.

More launch options can be found with

```bash
cargo run --release -- --help
```

![Simple Sim environment](./report/figures/screenshots/simple-sim-gui.png)
![Another simple sim enviornment](./report/figures/screenshots/simple_sim_depot.png)

## ROS 2

The [`multi_robot_control`](./ros_ws/src/multi_robot_control) ROS 2 package, contains the nodes needed to run the robot behaviors in ROS 2. [`ros_agent`](./ros_ws/src/multi_robot_control/src/ros_agent) is the main behavior node which manages the [`botbrain`](#botbrain) robot state.

The ROS 2 packages can be built with:

```bash
source /opt/ros/jazzy/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

And run with:

```bash
ros2 launch multi_robot_control multi_robot.launch.py behavior:=search robots:=0,0,0:2,0,1 map:=../worlds/bitmap/depot/depot.yaml world:=depot
```

This will launch Gazebo and Rviz2 windows, where the simulation can be observed.

See [ROS 2 Agent](./ros_ws/ros_agent.md) for more information.

![ROS 2 environment](./report/figures/screenshots/multi_robot_map_overlay.png)

### Dockerfile

A Dockerfile is provided to run ROS 2 and Gazebo. The provided [`enter-docker.sh`](./enter-docker.sh) can be used to build and enter the container.

```bash
./enter-docker.sh
```

To rebuild the container before running run

```bash
./enter-docker.sh --rebuild
```

## Trainer

The [`trainer`](./trainer) crate can be used to train models defined in [`botbrain`](#botbrain). It defines the training loop and implements deep reinforcement learning.

See [the trainer README](./trainer/trainer.md) for more information.

## Plotting

The [`botplot`](./botplot) python library is used to run [`simple_sim`](#simple_sim) and Gazebo in an automated manner to collect data, which can be plotted with the provided functions. Simulator runs are cached to avoid rerunning scenarios. The [`plotting`](./plotting) directory contains scripts which use `botplot` to run the simulators and create plots.

### Using [`botplot`](./botplot/)

Install the package

```bash
python3 -m venv venv      # Create virtual enviornment
source venv/bin/activate  # Activate the enviornment
pip install -e ./botplot  # Install the `botplot` package
```

Run a plotting script

```bash
python3 ./plotting/report/sim_consistency.py
```

### Compiling the LaTeX Report

```bash
cd report
latexmk -pdf -bibtex-cond -shell-escape -interaction=nonstopmode main.tex
```

## Nix Integration

This project contains a [`flake.nix`](./flake.nix) which allows self-contained build recipes and development shells. All dependencies are automatically downloaded.

#### Build the Project

Build the entire project:

- Executables: `simple_sim` and `trainer`
- Documentation: `botbrain` and `simple_sim`
- LaTeX Report

```bash
nix build .
```

The output is located in the `result` directory.

Sub-packages can be listed with the following

```bash
nix flake show
```

#### Development Shell

Use the following command to install dependencies and start the development shell. ROS 2 is not supported with this shell, the `Dockerfile` can be used instead.

```bash
nix develop
```
