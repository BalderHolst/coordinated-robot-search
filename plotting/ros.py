import botplot as bp
import math

def main():
    bp.seed(42)

    scenario = bp.Scenario(
        title = "ROS 2 Example",
        world = bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior = "search:pathing",
        duration = 30,
        robots = [bp.Robot(x=0, y=-2)],
    )

    res = bp.run_sim(scenario)
    bp.plot_paths(res, "Simple Sim Paths")

    res = bp.run_ros(scenario, headless=False)
    bp.plot_paths(res, "ROS 2 Paths")
    bp.plot_coverage(res, "ROS 2 Coverage")

if __name__ == "__main__":
    main()
