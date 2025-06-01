# ROS 2 Agent

This is `botbrain` implementation in ROS 2 with Gazebo simulation of the TurtleBot 4.

## Close all ROS 2 related processes

Sometimes ROS 2 processes are not closed properly, so you might need to kill them manually if weird things happen.

```bash
ps aux | grep -E 'rviz|gz|ros' | grep -v -E 'grep' | awk '{print $2}' | xargs kill
```

## Build

Build workspace with all rust executables, launch files and supporting files.
Make sure all prerequisites are installed (See [README](../README.md)) or use the provided Dockerfile.

```bash
cd ros_ws/
colcon build --symlink-install
source install/setup.bash
```

## See the robot

To see the robot in RViz 2 run:

```bash
ros2 launch tb4_describtion robot_rviz_description.launch.py
```

## Launch the multi-robot search

This launches the Gazebo simulator and Rviz 2 windows, where the simulation can be observed. Make sure the provided map and gazebo world are the same.

```bash
cd ros_ws/
ros2 launch multi_robot_control multi_robot.launch.py behavior:=search:pathing robots:=0,0,0:2,0,1 map:=../worlds/bitmap/depot/depot.yaml world:=depot
```

### Launching robots

The robots are spawned in the world by the `spawn_robots` launch file located in `tb4_description` package.

To spawn 2 randomly placed robots run:

```bash
ros2 launch multi_robot_control multi_robot.launch.py behavior:=search:pathing robots:=2
```

Or specify the location and angle of each robot with the following syntax:

```bash
ros2 launch tb4_description spawn_robots.launch.py behavior:=search:pathing robots:=0,0,0:2,0,1
```

This will spawn a robot at x=0, y=0, and yaw=0, and 1 robot at x=2, y=0, yaw=1.

### More launch options

For all launch parameters run:

```bash
ros2 launch multi_robot_control multi_robot.launch.py --show-args
```
