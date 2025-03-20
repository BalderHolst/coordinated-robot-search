import os
import matplotlib.pyplot as plt
import polars as pl

import botplot as bp

DATA_DIR = "data"
PLOT_DIR = "plots"

if not os.path.exists(DATA_DIR): os.makedirs(DATA_DIR)
if not os.path.exists(PLOT_DIR): os.makedirs(PLOT_DIR)

def plot(data_file: str, output_file: str):

    df = pl.read_parquet(data_file)

    plt.plot(df["time"], df["coverage"])
    plt.xlabel("Time")
    plt.ylabel("Coverage")
    plt.title("Coverage over time")
    plt.savefig(output_file)

    print(f"Plot saved to {output_file}")


if __name__ == "__main__":
    data_file = f"{DATA_DIR}/simple.parquet"

    scenario = bp.Scenario(
        world="../simple_sim/worlds/objectmap/small_empty.ron",
        behavior="search",
        duration=800,
        robots=[bp.Robot(), bp.Robot(x=1.0, angle=3)]
    )

    bp.run_sim(scenario, data_file)

    plot(data_file, f"{PLOT_DIR}/simple.png")

