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

    results = {}

    for behavior in BEHAVIORS:
        for n in range(3, len(ROBOTS)+1):
            print(f"\n=====> Running {behavior} with {n} robots <=====")

            scenario = bp.Scenario(
                title=f"behavior",
                world="simple_sim/worlds/objectmap/medium_obstacles.ron",
                behavior=behavior,
                duration=1000,
                robots=ROBOTS[:n],
            )


            results[f"many-{n}/{behavior}"] = bp.run_sim(scenario)

    for name, result in results.items():
        print("\n=====> Plotting", name, "<=====")
        bp.plot_paths(result, name + "/path/part", 10)
        bp.plot_spread(result, name + "/spread")
        bp.plot_coverage(result, name + "/coverage")
        bp.plot_performance(result, name + "/performance")

if __name__ == "__main__":
    main()
