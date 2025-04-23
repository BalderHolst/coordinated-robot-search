import botplot as bp

if __name__ == "__main__":
    scenario = bp.Scenario(
        title=f"robots",
        world="simple_sim/worlds/objectmap/medium_obstacles.ron",
        behavior="search",
        duration=1000,
        robots=[
            bp.Robot(x=4, y=4),
            bp.Robot(x=4, y=6),
            bp.Robot(x=6, y=4),
            bp.Robot(x=6, y=6),
            bp.Robot(x=8, y=4),
            bp.Robot(x=8, y=6),
        ],
    )

    res = bp.run_sim(f"path", scenario)

    bp.plot_paths(res, "paths", 10)
