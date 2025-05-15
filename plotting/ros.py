import botplot as bp


def main():
    robots = 4

    sim = bp.ros_ws.launch(
        "multi_robot_control",
        "multi_robot.launch.py",
        behavior="search:pure-pathing",
        n_robots=robots,
        headless=False,
        block=False,
        # capture_output=True,
    )

    bp.ros_ws.run("multi_robot_control", "data_logger", timeout=10, robot_count=robots)

    print("Killing simulation...")
    sim.kill()
    bp.utils.kill_gazebo()

if __name__ == "__main__":
    main()
