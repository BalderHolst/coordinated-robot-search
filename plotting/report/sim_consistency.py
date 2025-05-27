import botplot as bp


def main_circle():
    bp.seed(0)
    GZ_WORLD = "depot"

    scenario_ros2 = bp.Scenario(
        title="ROS 2",
        world=bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior="dumb:circle",
        duration=20,
        gazebo_world=GZ_WORLD,
        robots=[
            bp.Robot(x=-8, y=0.2),
        ],
    )
    res_ros2 = bp.run_ros(scenario_ros2, headless=False, use_cache=True)
    bp.plot_paths(res_ros2, "ROS 2 Paths Circle")

    scenario_simple = bp.Scenario(
        title="Simple Sim",
        world="worlds/bitmap/depot/depot.yaml",
        behavior="dumb:circle",
        duration=20,
        robots=[
            bp.Robot(x=-8, y=0.2),
        ],
    )
    res_simple = bp.run_sim(scenario_simple, use_cache=False)
    bp.plot_paths(res_simple, "Simple Sim Paths Circle")


def main_straight():
    bp.seed(0)
    GZ_WORLD = "depot"

    scenario_ros2 = bp.Scenario(
        title="ROS 2",
        world=bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior="dumb:straight",
        duration=20,
        gazebo_world=GZ_WORLD,
        robots=[
            bp.Robot(x=-8, y=0.2),
        ],
    )
    res_ros2 = bp.run_ros(scenario_ros2, headless=False, use_cache=False)
    bp.plot_paths(res_ros2, "ROS 2 Paths Straight")

    scenario_simple = bp.Scenario(
        title="Simple Sim",
        world="worlds/bitmap/depot/depot.yaml",
        behavior="dumb:straight",
        duration=20,
        robots=[
            bp.Robot(x=-8, y=0.2),
        ],
    )
    res_simple = bp.run_sim(scenario_simple, use_cache=False)
    bp.plot_paths(res_simple, "Simple Sim Paths Straight")


if __name__ == "__main__":
    main_circle()
    main_straight()
