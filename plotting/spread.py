import botplot as bp

if __name__ == "__main__":
    scenario = bp.Scenario(
        title=f"robots",
        world="worlds/objectmap/medium_obstacles.ron",
        behavior="search",
        duration=400,
        robots=[
            bp.Robot(x=4, y=4),
            bp.Robot(x=4, y=6),
            bp.Robot(x=6, y=4),
            bp.Robot(x=6, y=6),
            bp.Robot(x=8, y=4),
            bp.Robot(x=8, y=6),
        ],
    )

    res = bp.run_sim(scenario)

    bp.plot_spread(res, "spread")
