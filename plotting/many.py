import botplot as bp

ROBOTS = [
    bp.Robot(x=4, y=4),
    bp.Robot(x=4, y=6),
    bp.Robot(x=6, y=4),
    bp.Robot(x=6, y=6),
    bp.Robot(x=8, y=4),
    bp.Robot(x=8, y=6),
    bp.Robot(x=10, y=4),
    bp.Robot(x=10, y=6),
]

BEHAVIORS = [
    "avoid-obstacles:avoid-closest",
    "search:full",
    "search:naive-proximity",
    "search:no-proximity",
    "search:no-pathing",
    "search:pure-pathing",
]

def main():

    for behavior in BEHAVIORS:
        for n in range(3, len(ROBOTS)+1):
            print(f"=====> Running {behavior} with {n} robots <=====")

            scenario = bp.Scenario(
                title=f"behavior",
                world="simple_sim/worlds/objectmap/medium_obstacles.ron",
                behavior=behavior,
                duration=1000,
                robots=ROBOTS[:n],
            )

            res = bp.run_sim(scenario)

            bp.plot_paths(res, f"many-{n}/{behavior}", 10)

if __name__ == "__main__":
    main()
