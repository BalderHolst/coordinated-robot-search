import botplot as bp

world = "warehouse"


def main():
    bp.seed(42)

    scenario = bp.Scenario(
        title="ROS 2 Example",
        world=bp.repo_path(f"worlds/bitmap/{world}/{world}.yaml"),
        gazebo_world=world,
        behavior="search:pathing",
        duration=100,
        robots=1,
    )

    res1 = bp.run_sim(scenario)
    bp.plot_paths(res1, "Simple Sim Paths")

    res2 = bp.run_ros(scenario, headless=False)
    bp.plot_paths(res2, "ROS 2 Paths")

    bp.plorrort_coverage([res1, res2], "ROS 2 Coverage")


if __name__ == "__main__":
    main()
