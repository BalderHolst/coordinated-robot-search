import os
import subprocess
import matplotlib.pyplot as plt
import polars as pl

DATA_DIR = "data"
PLOT_DIR = "plots"

if not os.path.exists(DATA_DIR): os.makedirs(DATA_DIR)
if not os.path.exists(PLOT_DIR): os.makedirs(PLOT_DIR)

def run_sim(scenario: str, output: str):

    # Get simulator binary from $SIMULATOR environment variable
    simulator = os.environ.get("SIMULATOR")

    subprocess.run([simulator, "scenario", scenario, "-o", output, "--headless"])

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

    run_sim("../simple_sim/worlds/scenarios/simple.scenario.ron", data_file)

    plot(data_file, f"{PLOT_DIR}/simple.png")

