import botplot as bp

if __name__ == "__main__":
    scenario = bp.Scenario(
        title=f"non-compressed",
        world="simple_sim/worlds/objectmap/medium_obstacles.ron",
        behavior="search",
        duration=20,
        robots=[
            bp.Robot(x=4, y=4),
            bp.Robot(x=4, y=6),
            bp.Robot(x=6, y=4),
        ],
    )

    res = bp.run_sim(scenario)

    bp.plot_bytes(res, "bytes.png")
