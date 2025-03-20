import botplot as bp

if __name__ == "__main__":
    scenario = bp.Scenario(
        title=f"robots",
        world="simple_sim/worlds/objectmap/small_empty.ron",
        behavior="search",
        duration=400,
        robots=[bp.Robot(), bp.Robot(x=2)],
    )

    res = bp.run_sim(f"path", scenario)

    bp.plot_paths(res, "coverage.png")
