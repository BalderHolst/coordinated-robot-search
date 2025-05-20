import botplot as bp

def main():
    bp.seed(42)

    scenario = bp.Scenario(
        title = "ROS 2 Example",
        world = bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior = "search:pathing",
        duration = 30,
        robots = [bp.Robot(x=0, y=-2)],
    )

    res1 = bp.run_sim(scenario)
    bp.plot_paths(res1, "Simple Sim Paths")

    res2 = bp.run_ros(scenario, headless=False)
    bp.plot_paths(res2, "ROS 2 Paths")

    bp.plot_coverage([res1, res2], "ROS 2 Coverage")

if __name__ == "__main__":
    main()
