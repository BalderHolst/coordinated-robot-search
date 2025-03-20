import matplotlib.pyplot as plt
import polars as pl
import botplot as bp
from math import ceil, pi
import random

def plot(data_file: str, output_file: str):

    df = pl.read_parquet(data_file)

    plt.plot(df["time"], df["coverage"])
    plt.xlabel("Time")
    plt.ylabel("Coverage")
    plt.title("Coverage over time")
    plt.savefig(output_file)

    print(f"Plot saved to {output_file}")

if __name__ == "__main__":
    random.seed(42)

    results: list[bp.Result] = []

    for n in range(1, 11):
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
