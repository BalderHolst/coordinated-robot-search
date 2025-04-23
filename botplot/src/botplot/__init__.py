import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import subprocess
from dataclasses import dataclass
import polars as pl
import json
import hashlib

plt.rcParams.update({
    "text.usetex": True,        # Use LaTeX to render all text
    "font.family": "serif",     # Set font family to serif
    "font.serif": ["Computer Modern Roman"],  # Default LaTeX font
})

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

    def __str__(self) -> str:
        return self.to_ron()

OBJ_LABEL = "Objs"
BITMAP_LABEL = "Bitmap"
@dataclass
class World:
    data: dict

    def is_obj(self) -> bool:
        return OBJ_LABEL in self.data

    def is_bitmap(self) -> bool:
        return BITMAP_LABEL in self.data

    def desc(self) -> dict:
        if self.is_obj(): return self.data[OBJ_LABEL]
        if self.is_bitmap(): return self.data[BITMAP_LABEL]
        else: raise ValueError(f"World must be either a bitmap or an object world. Found keys: {self.data.keys()}")

    def dims(self) -> tuple[float, float]:
        desc = self.desc()
        if self.is_obj():
            return desc["width"], desc["height"]
        if self.is_bitmap():
            raise NotImplemented("Bitmap world not implemented yet.")

@dataclass
class Result:
    dataframe: str
    description: str

    @classmethod
    def from_stem(cls, stem: str) -> "Result":
        dataframe = os.path.join(data_dir(), f"{stem}.ipc")
        description = os.path.join(data_dir(), f"{stem}.json")

        if not os.path.exists(dataframe):
            print(f"Error: {dataframe} does not exist.")
            exit(1)

        if not os.path.exists(description):
            print(f"Error: {description} does not exist.")
            exit(1)

        return cls(dataframe, description)

    def df(self) -> pl.DataFrame:
        return pl.read_ipc(self.dataframe)

    def desc(self) -> dict:
        with open(self.description) as f:
            return json.load(f)

    def paths(self) -> tuple[str, str]:
        """Returns the releative paths to the data and description files."""
        data_path = os.path.relpath(self.dataframe, os.path.curdir)
        desc_path = os.path.relpath(self.description, os.path.curdir)
        return data_path, desc_path

    def world(self) -> World:
        return World(self.desc()["world"])

def relpath(path: str) -> str:
    """Returns the relative path to the given path."""
    return os.path.relpath(path, os.path.curdir)


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

def world_plot(fig, ax, result: Result, title: str, out_file: str):

    if title:
        ax.set_title(title, fontsize=16)

    width, height = result.world().dims()

    xmin, ymin = -width / 2, -height / 2
    xmax, ymax =  width / 2,  height / 2

    plt.margins(0.0)

    # Remove frame ticks
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')

    # ncol = min(len(ax.get_legend_handles_labels()[0]), 4)  # Get the number of legend entries
    # ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02), borderaxespad=0., ncol=ncol)

    plt.tight_layout()

    fig.savefig(out_file, dpi=300, bbox_inches='tight', pad_inches=0.2)


def run_sim(name: str, scenario: Scenario | str, headless: bool = True, use_cache=True) -> Result:

    hash = hashlib.sha256((name + str(scenario)).encode()).hexdigest()

    # Get simulator binary from $SIMULATOR environment variable
    simulator = sim_file()

    stem = f"{name}-{hash}"
    data_file = os.path.join(data_dir(), f"{stem}.ipc")
    desc_file = os.path.join(data_dir(), f"{stem}.json")

    if use_cache and os.path.exists(data_file) and os.path.exists(desc_file):
        print(f"[INFO] Using cached result:")
        print(f"    {stem}.ipc")
        print(f"    {stem}.json")
        return Result(data_file, desc_file)


    os.makedirs(os.path.dirname(data_file), exist_ok=True)

    flags = ["-o", data_file, "--description", desc_file]

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

    print(f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'")

    return Result(data_file, desc_file)

def plot_coverage(results: list[Result] | Result, output_file: str, title=None):

    if isinstance(results, Result): results = [results]

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
    plt.close()

    print(f"Plot saved to '{relpath(output_file)}'")

def plot_paths(result: Result, title: str, segments=1):
    df = result.df()
    desc = result.desc()


    for seg in range(segments):
        fig, ax = plt.subplots()

        t = df["time"]
        end = (len(t) // segments) * (seg + 1)
        try:
            for i, _ in enumerate(desc["robots"]):


                x = df[f"robot_{i}/x"]
                y = df[f"robot_{i}/y"]


                x = x[:end]
                y = y[:end]

                line, = ax.plot(x, y, label=f"Robot {i}")

                color = line.get_color()

                cross_size = 0.4
                widen = lambda x, size: [x - size, x + size]
                ax.plot(widen(x[0],  cross_size), widen(y[0], cross_size), color=color, linewidth=4, alpha=0.7)
                ax.plot(widen(x[0], -cross_size), widen(y[0], cross_size), color=color, linewidth=4, alpha=0.7)

                # Draw circle at starting position
                ax.add_patch(patches.Circle((x[-1], y[-1]), color=color, radius=0.5, alpha=0.7))

        except pl.exceptions.ColumnNotFoundError as e:
            data_path, desc_path = result.paths()
            print(f"Error using data in files:\n    {data_path}\n    {desc_path}\n")
            print(df)
            print(f"    => {e}")
            exit(1)

        file = os.path.join(plot_dir(), f"{title.replace(" ", "-").lower()}-{seg + 1}-of-{segments}.png")

        world_plot(fig, ax, result, f"{title} (after {t[end-1]:.0f}s)", file)


        print(f"Plot saved to '{relpath(file)}'")
