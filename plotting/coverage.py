import matplotlib.pyplot as plt
import polars as pl
import botplot as bp
from math import ceil, pi
import random

def run_and_plot():
    random.seed(42)

    results: list[bp.Result] = []

    for n in range(1, 3):
        print(f"Running simulation with {n} robots")
        robots = [
                bp.Robot(
                    x = i %  int(ceil(n)),
                    y = i // int(ceil(n)),
                    angle=random.random() * 2 * pi - pi
                ) for i in range(n)
        ]
        scenario = bp.Scenario(
            title=f"{n} robots",
            world="simple_sim/worlds/objectmap/small_empty.ron",
            behavior="search",
            duration=400,
            robots=robots,
        )

        results.append(bp.run_sim(f"test/{n}", scenario))

    bp.plot_coverage(results, "coverage.png")


if __name__ == "__main__":

    result = bp.Result.from_stem("out")
    bp.plot_coverage(result, "coverage.png")

