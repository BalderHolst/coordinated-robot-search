import botplot as bp

if __name__ == "__main__":
    scenario = bp.Scenario(
        title=f"robots",
        world="simple_sim/worlds/objectmap/medium_obstacles.ron",
        behavior="search",
        duration=400,
        robots=[
            bp.Robot(x=10, y=-20),
            bp.Robot(x=12, y=-20),
            bp.Robot(x=10, y=-22),
            bp.Robot(x=12, y=-22),
        ],
    )

    res = bp.run_sim(f"path", scenario)

    bp.plot_paths(res, "paths", 10)
