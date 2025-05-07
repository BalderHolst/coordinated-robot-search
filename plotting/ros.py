import botplot as bp


def main():
    bp.ros_ws.launch(
        "multi_robot_control",
        "multi_robot.launch.py",
        behavior="avoid-obstacles",
        headless=True,
        n_robots=4,
    )


if __name__ == "__main__":
    main()
