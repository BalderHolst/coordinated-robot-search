import botplot as bp


def main_dumb():
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
    bp.plot_paths(res_ros2, "ROS 2 Paths")

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
    bp.plot_paths(res_simple, "Simple Sim Paths")


def main_search():
    bp.seed(0)

    scenario_ros2 = bp.Scenario(
        title="ROS 2",
        world=bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior="search:pathing",
        duration=180,
        robots=[bp.Robot(x=-8, y=0.2)],
    )
    res_ros2 = bp.run_ros(scenario_ros2, headless=False)
    bp.plot_paths(res_ros2, "ROS 2 Paths")

    scenario_simple = bp.Scenario(
        title="Simple Sim",
        world="worlds/bitmap/depot/depot.yaml",
        behavior="search:pathing",
        duration=180,
        robots=[
            bp.Robot(x=-8, y=0.2),
        ],
    )
    res_simple = bp.run_sim(scenario_simple)
    bp.plot_paths(res_simple, "Simple Sim Paths")


if __name__ == "__main__":
    main_dumb()
    # main_search()
