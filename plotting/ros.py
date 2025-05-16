import botplot as bp


def main():
    bp.seed(0)

    scenario = bp.Scenario(
        title = "ROS 2 Example",
        world = bp.repo_path("simple_sim/worlds/bitmap/depot/depot.yaml"),
        behavior = "search:pure-pathing",
        duration = 400,
        robots = 2,
    )

    # res = bp.run_sim(scenario)
    # bp.plot_paths(res, "Simple Sim Paths")

    res = bp.run_ros(scenario, headless=False)
    bp.plot_paths(res, "ROS 2 Paths")
    bp.plot_coverage(res, "ROS 2 Coverage")

if __name__ == "__main__":
    main()
