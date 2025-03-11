# Good commands for ROS2

Kill all

```bash
ps aux | grep -E 'rviz|gz|ros' | grep -v -E 'home|grep' | awk '{print $2}' | xargs kill
```

## General

Make package under `ros2_ws/src`

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

Build workspace

```bash
colcon build --symlink-install
```

- `symlink-install` allows the installed files to be changed by changing
  the files in the source space (e.g. Python files or other non-compiled resources)
  for faster iteration.

  Build everything for a package

```bash
colcon build --packages-up-to YOUR_PACKAGE
```

Install dependencies from `ros2_ws`

```bash
rosdep install --from-paths src -y --ignore-src
```

## NAV2

### Map

Manually save a map to a file

```bash
ros2 run nav2_map_server map_saver_cli -f path/filename
```

## TF2

PDF to see published TF2 frames

```bash
ros2 run tf2_tools view_frames
```

Convert xacro file to URDF:

```bash
ros2 run xacro xacro --inorder your_robot.xacro > your_robot.urdf
```
