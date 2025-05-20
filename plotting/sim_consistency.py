import botplot as bp


def main():
    bp.seed(0)

    scenario_ros2 = bp.Scenario(
        title="ROS 2",
        world=bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior="dumb:circle",
        duration=30,
        robots=1,
    )
    res_ros2 = bp.run_ros(scenario_ros2, headless=False)
    bp.plot_paths(res_ros2, "ROS 2 Paths")

    scenario_simple = bp.Scenario(
        title="Simple Sim",
        world="worlds/bitmap/depot/depot.yaml",
        behavior="dumb:circle",
        duration=30,
        robots=[
            bp.Robot(x=0, y=0),
        ],
    )
    res_simple = bp.run_sim(scenario_simple)
    bp.plot_paths(res_simple, "Simple Sim Paths")


if __name__ == "__main__":
    main()
