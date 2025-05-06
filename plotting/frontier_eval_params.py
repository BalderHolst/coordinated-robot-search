import botplot as bp
from math import ceil, pi
import random


def main():
    behaviors = [
        "pure-pathing-s33-d33-t33",
        "pure-pathing-s10-d30-t60",
        "pure-pathing-s80-d10-t10",
        "pure-pathing-s10-d80-t10",
        "pure-pathing-s10-d10-t80",
        "pure-pathing-s20-d60-t20",
        "pure-pathing-s10-d45-t45",
        "pure-pathing-s0-d50-t50",
        "pure-pathing-s100-d0-t0",
        "pure-pathing-s0-d100-t0",
        "pure-pathing-s0-d0-t100",
    ]
    worlds = [
        "simple_sim/worlds/bitmap/depot/depot.yaml",
        # "simple_sim/worlds/objectmap/medium_obstacles.ron",
    ]

    random.seed(42)

    n = 1
    robots = [
        bp.Robot(
            x=i % int(ceil(n)),
            y=i // int(ceil(n)),
            angle=random.random() * 2 * pi - pi,
        )
        for i in range(n)
    ]

    for world in worlds:
        world_name = world.split("/")[-1].split(".")[0]
        results: list[bp.Result] = []
        for behavior in behaviors:
            scenario = bp.Scenario(
                title=f"{world_name}: {n} robots: {behavior}",
                world=world,
                behavior="search:" + behavior,
                duration=200,
                robots=robots,
            )

            res = bp.run_sim(scenario)
            results.append(res)

        bp.plot_coverage(results, f"frontier_eval_params_{world_name}")


if __name__ == "__main__":
    main()
