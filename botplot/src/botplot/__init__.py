import os
import subprocess
from dataclasses import dataclass
import polars as pl
import json
import matplotlib.pyplot as plt

@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0

@dataclass
class Scenario:
    title: str
    world: str
    behavior: str
    duration: int
    robots: list[Robot]

    def to_ron(self) -> str:
        robots = ", ".join([f"(pos: (x: {r.x}, y: {r.y}), angle: {r.angle})" for r in self.robots])

        return f"""
        Scenario(
            title: "{self.title}",
            world: "{self.world}",
            behavior: "{self.behavior}",
            duration: {self.duration},
            robots: [{robots}]
        )
        """

@dataclass
class Result:
    dataframe: str
    description: str

    def df(self) -> pl.DataFrame:
        return pl.read_parquet(self.dataframe)

    def desc(self) -> dict:
        with open(self.description) as f:
            return json.load(f)

def env(var: str) -> str:
    path = os.environ.get(var)
    if not path:
        print(f"Error: ${var} environment variable is not set.")
        exit(1)
    return path

def env_dir(var: str) -> str:
    path = env(var)
    if not os.path.exists(path):
        os.makedirs(path)
    if not os.path.isdir(path):
        print(f"Error: ${var} environment variable is not set to a directory path.")
        exit(1)
    return path

def env_file(var: str) -> str:
    path = env(var)
    if not os.path.isfile(path):
        print(f"Error: ${var} environment variable is not set to a file path.")
        exit(1)
    return path

def data_dir() -> str: return env_dir("DATA_DIR")
def plot_dir() -> str: return env_dir("PLOT_DIR")
def sim_file() -> str: return env_file("SIMULATOR")

def run_sim(name: str, scenario: Scenario | str, headless: bool = True) -> tuple[str, str]:

    # Get simulator binary from $SIMULATOR environment variable
    simulator = sim_file()

    output = os.path.join(data_dir(), f"{name}.parquet")
    desc_output = os.path.join(data_dir(), f"{name}.json")

    os.makedirs(os.path.dirname(output), exist_ok=True)

    flags = ["-o", output, "--description", desc_output]

    if headless: flags.append("--headless")

    if isinstance(scenario, Scenario):
        s = scenario.to_ron()
        print(s)
        subprocess.run([simulator, "scenario", "-"] + flags, input=s, text=True)
    elif isinstance(scenario, str):
        subprocess.run([simulator, "scenario", scenario] + flags)
    else:
        print("Error: scenario must be a `Scenario` object or a string. Found: ", type(scenario))
        exit(1)

    print(f"Simulation output saved to './{os.path.relpath(output, os.path.curdir)}'")

    return Result(output, desc_output)

def plot_coverage(results: list[Result], output_file: str, title=None):
    for result in results:
        df = result.df()
        desc = result.desc()
        plt.plot(df["time"], df["coverage"], label=desc["title"])

    if title is None: title = "Coverage over time"

    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title(title)
    plt.legend()

    output_file = os.path.join(plot_dir(), output_file)

    plt.savefig(output_file)

    print(f"Plot saved to '{output_file}'")
