import botplot as bp
from math import ceil, pi
import random


def main():
    behaviors = [
        "pure-pathing-s33-d33-t33",
        "pure-pathing-s33-d33-t33",
        "pure-pathing-s10-d30-t60",
        "pure-pathing-s80-d10-t10",
        "pure-pathing-s10-d80-t10",
        "pure-pathing-s10-d10-t80",
        "pure-pathing-s20-d60-t20",
    ]

    random.seed(42)

    results: list[bp.Result] = []

    n = 5
    robots = [
        bp.Robot(
            x=i % int(ceil(n)),
            y=i // int(ceil(n)),
            angle=random.random() * 2 * pi - pi,
        )
        for i in range(n)
    ]

    for behavior in behaviors:
        print(f"Running simulation with {n} robots with {behavior}")
        scenario = bp.Scenario(
            title=f"{n} robots: {behavior}",
            world="simple_sim/worlds/bitmap/depot/depot.yaml",
            behavior=behavior,
            duration=400,
            robots=robots,
        )

        res = bp.run_sim(scenario, f"test/{behavior}")
        results.append(res)

    # TODO: Make plotting of results
    # bp.plot_frontier_eval_params(results, "frontier_eval_params.png")


if __name__ == "__main__":
    main()
