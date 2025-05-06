import sys
import botplot as bp

ROBOTS = [
    bp.Robot(x=2, y=4),
    bp.Robot(x=2, y=6),
    bp.Robot(x=4, y=4),
    bp.Robot(x=4, y=6),
    bp.Robot(x=6, y=4),
    bp.Robot(x=6, y=6),
    bp.Robot(x=8, y=4),
    bp.Robot(x=8, y=6),
]

BEHAVIORS = [
    "avoid-obstacles:avoid-closest",
    "search:full",
    "search:no-pathing",
    "search:pure-pathing",
]

DURATION = 1600
N = [3, 4, 5, 6, 7, 8]

def main():

    force = False

    [_, *args] = sys.argv
    for arg in args:
        match arg:
            case "--force" | "-f": force = True
            case _:
                print(f"Unknown argument: {arg}")
                sys.exit(1)

    results = {}

    for behavior in BEHAVIORS:
        for n in N:
            print(f"\n=====> Running {behavior} with {n} robots <=====")

            scenario = bp.Scenario(
                title=f"{behavior}",
                world="simple_sim/worlds/objectmap/medium_obstacles.ron",
                behavior=behavior,
                duration=DURATION,
                robots=ROBOTS[:n],
            )

            results[f"many-{n}/{behavior}"] = bp.run_sim(scenario)

    for name, result in results.items():
        print("\n=====> Plotting", name, "<=====")
        bp.plot_coverage(result, name + "/coverage", force=force)
        bp.plot_performance(result, name + "/performance", force=force)
        bp.plot_spread(result, name + "/spread", force=force)
        bp.plot_paths(result, name + "/path/part", 10, force=force)

    for n in N:
        dir = f"many-{n}"
        runs = list(map(lambda b: results[f"{dir}/{b}"], BEHAVIORS))

        print("\n=====> Plotting Combined", dir, "<=====")
        bp.plot_coverage(runs, dir + "/coverage", force=force)
        bp.plot_performance(runs, dir + "/performance", max = 12, force=force)
        bp.plot_spread(runs, dir + "/spread", force=force)


if __name__ == "__main__":
    main()
