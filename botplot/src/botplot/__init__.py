import os
import json
import hashlib
import random
import time
from math import sqrt
from dataclasses import dataclass
from typing import Self

import polars as pl
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

from botplot.rust_crate import RustCrate
from botplot.colcon_workspace import ColconWorkspace
import botplot.utils as utils

import botplot.colors as colors
from botplot.colors import COLORS

mpl.rcParams["axes.prop_cycle"] = mpl.cycler(color=COLORS)

plt.rcParams.update(
    {
        "text.usetex": utils.check_installed("latex"),  # Use LaTeX to render all text
        "font.family": "serif",  # Set font family to serif
        "font.serif": ["Computer Modern Roman"],  # Default LaTeX font
    }
)

mpl.rcParams["axes.edgecolor"] = "#888888"  # gray frame around the plot
mpl.rcParams["xtick.color"] = "#555555"  # gray x ticks
mpl.rcParams["ytick.color"] = "#555555"  # gray y ticks
mpl.rcParams["axes.labelcolor"] = "#444444"  # gray axis labels


def seed(seed):
    random.seed(seed)


GIT_DIR = ".git"


def root_dir() -> str:
    """Find the root directory of the project by looking for the .git directory."""
    # Check if the current directory is the root
    if os.path.exists(os.path.join(os.path.curdir, GIT_DIR)):
        return os.path.curdir

    # Traverse up the directory tree until we find the .git directory
    current_dir = os.path.abspath(os.path.curdir)
    while True:
        if os.path.exists(os.path.join(current_dir, GIT_DIR)):
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            raise FileNotFoundError(
                f"{GIT_DIR} directory not found in any parent directories"
            )
        current_dir = parent_dir


def relpath(path: str) -> str:
    """Return the relative path to the given path from the current directory."""
    return os.path.relpath(path, os.path.curdir)


def repo_path(*path: str) -> str:
    return os.path.join(os.path.abspath(root_dir()), *path)


def data_dir(*file):
    return repo_path("data", *file)


def plot_dir(*file):
    return repo_path("plots", *file)


botbrain = RustCrate(repo_path("botbrain"))
simple_sim = RustCrate(repo_path("simple_sim"), dependencies=[botbrain])
trainer = RustCrate(repo_path("trainer"), dependencies=[simple_sim, botbrain])

ros_ws = ColconWorkspace(repo_path("ros_ws"))


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
    gazebo_world: str | None = None

    def __init__(
        self,
        title: str,
        world: str,
        behavior: str,
        duration: int,
        robots: list[Robot] | int,
        gazebo_world: str | None = None,
    ):
        self.title = title
        self.world = world
        self.behavior = behavior
        self.duration = duration
        self.gazebo_world = gazebo_world

        if isinstance(robots, int):
            robots = place_robots(world, robots)

        self.robots = robots

    def to_ron(self) -> str:
        robots = ", ".join(
            [f"(pos: (x: {r.x}, y: {r.y}), angle: {r.angle})" for r in self.robots]
        )

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
    file: str
    data: dict

    @classmethod
    def from_description_file(cls, file: str) -> Self:
        if file.endswith(".ron") or file.endswith(".yaml"):
            abs_file = os.path.abspath(file)
            hash = hashlib.sha256(abs_file.encode()).hexdigest()
            name = os.path.basename(file)
            json_file = os.path.join(data_dir(), f"{name}-{hash}-desc.json")

            os.makedirs(os.path.dirname(json_file), exist_ok=True)

            if os.path.exists(json_file):
                print(f"Using cached json world description: {relpath(json_file)}")
            else:
                trainer.run(
                    [
                        "world-to-json",
                        "--input",
                        file,
                        "--output",
                        json_file,
                        "--force",
                    ]
                )
            file = json_file

        with open(file) as f:
            data = json.load(f)

        if "world" in data:
            data = data["world"]

        return cls(file, data)

    def is_obj(self) -> bool:
        return OBJ_LABEL in self.data

    def is_bitmap(self) -> bool:
        return BITMAP_LABEL in self.data

    def desc(self) -> dict:
        if self.is_obj():
            return self.data[OBJ_LABEL]
        if self.is_bitmap():
            return self.data[BITMAP_LABEL]
        else:
            raise ValueError(
                f"World must be either a bitmap or an object world. Found keys: {self.data.keys()}"
            )

    def img(self):
        desc_name = os.path.basename(self.file).replace(".json", "-world-desc.json")
        desc_file = os.path.join(data_dir(), desc_name)

        json.dump(self.desc(), open(desc_file, "w"), indent=4)

        img_name = os.path.basename(self.file).replace(".json", ".png")
        img_file = os.path.join(data_dir(), img_name)

        if os.path.exists(img_file):
            print(f"Using cached world image: {relpath(img_file)}")
        else:
            trainer.run(
                [
                    "world-to-img",
                    "--input",
                    desc_file,
                    "--output",
                    img_file,
                    "--theme",
                    "grayscale",
                    "--force",
                ]
            )

        print(f"World image saved to '{relpath(img_file)}'")

        return plt.imread(img_file)

    def dims(self) -> tuple[float, float]:
        desc = self.desc()
        if self.is_obj():
            return desc["width"], desc["height"]
        if self.is_bitmap():
            pgm_path = desc["image"]
            if not os.path.exists(pgm_path):
                print(f"Error: Bitmap image '{pgm_path}' does not exist.")
                exit(1)
            with open(os.path.join(pgm_path), "rb") as f:
                # Read the first line (format identifier, e.g., P2 or P5)
                _ = f.readline().strip()
                # Read the width and height from the next line
                width, height = map(int, f.readline().split())

            resolution = desc["resolution"]

            return width * resolution, height * resolution


@dataclass
class Result:
    dataframe_file: str
    description_file: str

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
        return pl.read_ipc(self.dataframe_file)

    def df_with_spread_file(self) -> str:
        stem = os.path.basename(self.dataframe_file).replace(".ipc", "")
        return os.path.join(data_dir(), f"{stem}-with-spread.ipc")

    def df_with_spread(self) -> pl.DataFrame:
        file = self.df_with_spread_file()
        if os.path.exists(file):
            print(f"Using cached spread file: {relpath(file)}")
            return pl.read_ipc(file)

        desc = self.desc()
        df = self.df()

        # Add spread column
        spread = pl.Series("spread-sd", [0.0] * len(df))
        center_x = pl.Series("center_x", [0.0] * len(df))
        center_y = pl.Series("center_y", [0.0] * len(df))

        n = len(desc["robots"])

        for i in range(len(df)):
            total_x = 0.0
            total_y = 0.0

            for id in range(n):
                x = df[f"robot_{id}/x"][i]
                y = df[f"robot_{id}/y"][i]
                total_x += x
                total_y += y

            mu_x = total_x / n
            mu_y = total_y / n

            center_x[i] = mu_x
            center_y[i] = mu_y

            total_dist = 0.0
            for id in range(n):
                x = df[f"robot_{id}/x"][i]
                y = df[f"robot_{id}/y"][i]
                dist = sqrt((x - mu_x) ** 2 + (y - mu_y) ** 2)
                total_dist += dist

            spread[i] = total_dist / n

        df = df.with_columns([center_x, center_y, spread])

        # Save the dataframe with spread
        df.write_ipc(file)

        return df

    def desc(self) -> dict:
        with open(self.description_file) as f:
            return json.load(f)

    def title(self) -> str:
        return self.desc()["title"]

    def paths(self) -> tuple[str, str]:
        """Returns the releative paths to the data and description files."""
        data_path = os.path.relpath(self.dataframe_file, os.path.curdir)
        desc_path = os.path.relpath(self.description_file, os.path.curdir)
        return data_path, desc_path

    def world(self) -> World:
        return World.from_description_file(self.description_file)


class ResultCollection:
    name: str
    results: list[Result]
    min: str
    max: str
    avg: str
    std: str

    def __init__(self, name: str, results: list[Result]) -> None:
        self.name = name
        self.results = results
        self.populate_dfs()

    def hash(self) -> str:
        """Returns a hash of the results in this collection."""
        hashes = [hashlib.sha256(r.dataframe_file.encode()).hexdigest() for r in self.results]
        return hashlib.sha256("".join(hashes).encode()).hexdigest()

    def populate_dfs(self):
        hash = self.hash()

        self.min = data_dir(f"{self.name}-min-{hash}.ipc")
        self.max = data_dir(f"{self.name}-max-{hash}.ipc")
        self.avg = data_dir(f"{self.name}-avg-{hash}.ipc")
        self.std = data_dir(f"{self.name}-std-{hash}.ipc")

        cashed = True
        if not os.path.exists(self.min): cashed = False
        if not os.path.exists(self.max): cashed = False
        if not os.path.exists(self.avg): cashed = False
        if not os.path.exists(self.std): cashed = False
        if cashed:
            print(f"Using cached collection results for '{self.name}':")
            print(f"    {relpath(self.min)}")
            print(f"    {relpath(self.max)}")
            print(f"    {relpath(self.avg)}")
            print(f"    {relpath(self.std)}")
            return

        dfs = [r.df() for r in self.results]

        l = len(dfs[0])

        cols = dfs[0].columns
        for df in dfs:
            if set(cols) != set(df.columns):
                print("\nError: Result columns do not match")
                exit(1)
            if l > len(df):
                l = len(df)

        # Trim dfs to the same length
        for i, df in enumerate(dfs):
            dfs[i] = df.head(l)

        min_df = pl.DataFrame()
        max_df = pl.DataFrame()
        avg_df = pl.DataFrame()
        std_df = pl.DataFrame()

        for col in cols:
            runs = [df[col] for df in dfs]
            df_temp = pl.DataFrame({f"s{i}": s for i, s in enumerate(runs)})

            max = df_temp.max_horizontal()
            min = df_temp.min_horizontal()
            avg = df_temp.mean_horizontal()

            std = df_temp.map_rows(
                lambda row: sqrt(
                    sum((x - sum(row) / len(row)) ** 2 for x in row) / len(row)
                ),
                return_dtype=pl.Float64,
            )["map"]

            max_df = max_df.with_columns((max).alias(col))
            min_df = min_df.with_columns((min).alias(col))
            avg_df = avg_df.with_columns((avg).alias(col))
            std_df = std_df.with_columns((std).alias(col))

        # Save the dataframes
        min_df.write_ipc(self.min)
        max_df.write_ipc(self.max)
        avg_df.write_ipc(self.avg)
        std_df.write_ipc(self.std)

    def min_df(self) -> pl.DataFrame: return pl.read_ipc(self.min)
    def max_df(self) -> pl.DataFrame: return pl.read_ipc(self.max)
    def avg_df(self) -> pl.DataFrame: return pl.read_ipc(self.avg)
    def std_df(self) -> pl.DataFrame: return pl.read_ipc(self.std)

    def plot(self, col: str, ax, spread=True, color=None, label: bool = True):
        avg = self.avg_df()
        std = self.std_df()

        time = avg["time"]

        l = None
        if label:
            l = self.name

        line, = ax.plot(time, avg[col], linewidth=2, label=l, color=color)
        if spread:
            color = line.get_color()
            top = avg[col] + std[col]
            bot = avg[col] - std[col]
            ax.fill_between(time, bot, top, color=color, alpha=0.15)
            ax.plot(time, bot, color=color, alpha=0.5, linewidth=0.5)
            ax.plot(time, top, color=color, alpha=0.5, linewidth=0.5)
        return line

    def with_spread(self) -> Self:
        dfs = [r.df_with_spread() for r in self.results]
        self.populate_dfs(dfs)
        return self

    def with_name(self, name: str) -> Self:
        self.name = name
        return self


class SimpleResult(Result):
    """Result from running a scenario with simple_sim."""


class ROSResult(Result):
    """Result from running a scenario with ROS."""


def run_sim(
    scenario: Scenario | str,
    headless: bool = True,
    use_cache=True,
    no_debug_soup: bool = False,
) -> SimpleResult:
    match scenario:
        case Scenario():
            name = scenario.title
        case str():
            name = scenario.replace("/", "-")
        case _:
            raise ValueError("Scenario must be either a string or a Scenario object.")

    name = name.replace(" ", "-").lower() + "-simple"

    hash = hashlib.sha256((name + str(scenario)).encode()).hexdigest()

    stem = f"{name}-{hash}"
    data_file = data_dir(f"{stem}.ipc")
    desc_file = data_dir(f"{stem}.json")

    # Ensure the directories exist
    os.makedirs(os.path.dirname(data_file), exist_ok=True)
    os.makedirs(os.path.dirname(desc_file), exist_ok=True)

    if use_cache and os.path.exists(data_file) and os.path.exists(desc_file):
        print(f"[INFO] Using cached result:")
        print(f"    {stem}.ipc")
        print(f"    {stem}.json")
        return SimpleResult(data_file, desc_file)

    os.makedirs(os.path.dirname(data_file), exist_ok=True)

    flags = ["-o", data_file, "--description", desc_file]
    if headless:
        flags.append("--headless")
    if no_debug_soup:
        flags.append("--no-debug-soup")

    if isinstance(scenario, Scenario):
        s = scenario.to_ron()
        print(s)
        simple_sim.run(["scenario", "-"] + flags, input=s, text=True)
    elif isinstance(scenario, str):
        if not os.path.exists(scenario):
            print(f"Error: Scenario file '{scenario}' does not exist.")
            exit(1)
        simple_sim.run(["scenario", scenario] + flags)
    else:
        print(
            "Error: scenario must be a `Scenario` object or a string. Found: ",
            type(scenario),
        )
        exit(1)

    print(
        f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'"
    )

    return SimpleResult(data_file, desc_file)


def interpolated_resample_reference_time(results: list[Result]) -> Result:
    # Find the result with the most time points
    max_length = 0
    max_idx = 0
    for i, result in enumerate(results):
        df = result.df()
        if len(df) > max_length:
            max_length = len(df)
            max_idx = i

    return results[max_idx]


def interpolated_resample_coverage(
    results: list[Result], reference_time: Result
) -> list[Result]:
    # Get the time points from the longest result
    reference_times = reference_time.df()["time"].to_list()
    reference_length = len(reference_times)

    # Create new resampled results
    resampled_results = []

    for result in results:
        df = result.df()
        # Skip if already matches the reference timestamps
        if len(df) == reference_length and df["time"].to_list() == reference_times:
            resampled_results.append(result)
            continue

            # Example (adapt to your variables):
        time = df["time"]
        coverage = df["coverage"]

        # Interpolate ROS coverage to match Simple Sim time points
        interpolated_coverage = np.interp(
            reference_times,  # target time points (float64)
            time.cast(
                pl.Float64
            ),  # source time points (cast to float64 for matching types)
            coverage,  # source coverage values
        )

        # Create new aligned ROS DataFrame
        interpolated_df = pl.DataFrame(
            {"time": reference_times, "coverage": interpolated_coverage}
        )

        stem = os.path.basename(result.dataframe_file).replace(".ipc", "-resampled")
        new_data_file = data_dir(f"{stem}.ipc")
        interpolated_df.write_ipc(new_data_file)
        resampled_results.append(Result(new_data_file, result.description_file))

    return resampled_results


def run_ros(
    scenario: Scenario | str, headless: bool = True, use_cache=True
) -> ROSResult:
    match scenario:
        case Scenario():
            name = scenario.title
        case str():
            name = scenario.replace("/", "-")
        case _:
            raise ValueError("Scenario must be either a string or a Scenario object.")

    name = name.replace(" ", "-").lower() + "-ros"

    hash = hashlib.sha256((name + str(scenario)).encode()).hexdigest()

    stem = f"{name}-{hash}"
    data_file = data_dir(f"{stem}.ipc")
    desc_file = data_dir(f"{stem}.json")

    # Ensure the directories exist
    os.makedirs(os.path.dirname(data_file), exist_ok=True)
    os.makedirs(os.path.dirname(desc_file), exist_ok=True)

    if use_cache and os.path.exists(data_file) and os.path.exists(desc_file):
        print(f"[INFO] Using cached result:")
        print(f"    {stem}.ipc")
        print(f"    {stem}.json")
        return ROSResult(data_file, desc_file)

    # Save the scenario description to json a file
    ron_desc_file = data_dir(f"{stem}.ron")
    with open(ron_desc_file, "w") as f:
        f.write(scenario.to_ron())
    simple_sim.run(["world-to-json", ron_desc_file, desc_file])

    print(f"Saved scenario description to '{relpath(desc_file)}'")

    os.makedirs(os.path.dirname(data_file), exist_ok=True)

    proc = None

    if isinstance(scenario, Scenario):
        s = scenario.to_ron()

        if not scenario.gazebo_world:
            print(f"Error: Scenario must have a gazebo world to be run in gazebo.")
            exit(1)

        print(s)

        robots = ":".join([f"{r.x},{r.y},{r.angle}" for r in scenario.robots])

        proc = ros_ws.launch(
            "multi_robot_control",
            "multi_robot.launch.py",
            block=False,
            behavior=scenario.behavior,
            robots=robots,
            map=scenario.world,
            headless=headless,
            use_rviz=not headless,
            world=scenario.gazebo_world,
        )

        ros_ws.run(
            "multi_robot_control",
            "data_logger",
            timeout=scenario.duration,
            robot_count=len(scenario.robots),
            output=data_file,
        )

    elif isinstance(scenario, str):
        raise NotImplemented("ROS simulation does not support string scenarios.")
    else:
        print(
            "Error: scenario must be a `Scenario` object or a string. Found: ",
            type(scenario),
        )
        exit(1)

    print("Killing simulation...")
    proc.kill()
    utils.kill_gazebo()

    time.sleep(2)  # Wait a bit for gazebo to shut down

    print(
        f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'"
    )

    return ROSResult(data_file, desc_file)


def place_robots_data(world: str | World, n: int) -> dict:
    if isinstance(world, World):
        world = world.file

    flags = []

    proc = trainer.run(
        [
            "place-robots",
            "--world",
            world,
            "-n",
            str(n),
            "--seed",
            str(random.randint(0, 2**64 - 1)),
        ]
        + flags,
        capture_output=True,
        text=True,
    )

    data = json.loads(proc.stdout)
    robots = []
    for pose in data["poses"]:
        x = pose["pos"]["x"]
        y = pose["pos"]["y"]
        angle = pose["angle"]
        robots.append(Robot(x, y, angle))

    data["robots"] = robots

    return data


def place_robots(world: str | World, n: int) -> list[Robot]:
    data = place_robots_data(world, n)
    return data["robots"]


def plot_world(
    fig, ax, world: World, title: str, out_file: str, borders: bool, plot_title: bool
) -> str:
    if title and plot_title:
        ax.set_title(title, fontsize=16)

    width, height = world.dims()

    xmin, ymin = -width / 2, -height / 2
    xmax, ymax = width / 2, height / 2

    if not borders:
        for spine in ax.spines.values():
            spine.set_visible(False)

    plt.margins(0.0)

    # Remove frame ticks
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymax, ymin)
    ax.set_aspect("equal")

    world_img = world.img()
    ax.imshow(world_img, extent=[xmin, xmax, ymin, ymax], origin="lower", zorder=0)

    plt.tight_layout()

    out_file = os.path.join(plot_dir(), f"{title.replace(' ', '-').lower()}.png")

    save_figure(fig, out_file)

    return out_file


def save_figure(fig, output_file: str):
    name = os.path.basename(output_file).replace(" ", "-").lower()
    dir = os.path.dirname(output_file)
    os.makedirs(dir, exist_ok=True)
    output_file = os.path.join(dir, name)
    fig.savefig(output_file, dpi=300, bbox_inches="tight", pad_inches=0.2)
    plt.close(fig)
    print(f"Plot saved to '{relpath(output_file)}'")
    return output_file


def plot_coverage(
    results: Result | list[Result] | ResultCollection | list[ResultCollection],
    name: str,
    title=None,
) -> str:
    fig, ax = plt.subplots(figsize=(9, 5))

    file = os.path.join(plot_dir(), f"{name}.png")

    if not isinstance(results, list):
        results = [results]

    if isinstance(results, list) and all(
        isinstance(item, ResultCollection) for item in results
    ):
        lines = [(c, c.plot("coverage", ax, label=False)) for c in results]
        for c, line in lines:
            c.plot("coverage", ax, color=line.get_color(), spread=False)

    else:
        for result in results:
            df = result.df()
            desc = result.desc()
            ax.plot(df["time"], df["coverage"], label=desc["title"])

    if len(results) > 1:
        ax.legend()

    if title is None:
        title = name

    ax.set_xlabel(r"Time (s)")
    ax.set_ylabel(r"Coverage (\%)")
    ax.set_title(title)

    return save_figure(fig, file)


def plot_avg_coverage_diff(results: list[tuple[pl.DataFrame, str]], title: str) -> str:
    file = os.path.join(plot_dir(), f"{title}.png")
    fig, ax = plt.subplots()
    ax.axhline(0, color="gray", linestyle="--", linewidth=1)
    for df, behavior in results:
        ax.plot(df["time"], df["diff"], label=behavior)
    if len(results) > 1:
        ax.legend()
    ax.set_xlabel(r"Time (s)")
    ax.set_ylabel(r"Coverage Diff (\% points)")
    ax.set_title(title)
    return save_figure(fig, file)


def show_velocities(
    ax, df: pl.DataFrame, robot_count: int, label: str | None = None
) -> pl.DataFrame:
    df = df.with_columns([pl.col("time").diff().alias("delta_time")])

    for i in range(robot_count):
        df = df.with_columns(
            [
                (
                    pl.col(f"robot_{i}/x").diff().pow(2)
                    + pl.col(f"robot_{i}/y").diff().pow(2)
                )
                .sqrt()
                .alias(f"robot_{i}/distance"),
            ]
        ).with_columns(
            [
                (pl.col(f"robot_{i}/distance") / pl.col("delta_time")).alias(
                    f"robot_{i}/vel"
                )
            ]
        )

    velocity_cols = [pl.col(f"robot_{i}/vel") for i in range(robot_count)]

    df = df.with_columns(
        [
            pl.min_horizontal(velocity_cols).alias("vel_min"),
            pl.max_horizontal(velocity_cols).alias("vel_max"),
            pl.mean_horizontal(velocity_cols).alias("vel_mean"),
        ]
    )

    (line,) = ax.plot(df["time"], df["vel_mean"], linewidth=2, label=label)
    color = line.get_color()
    ax.fill_between(
        df["time"],
        df["vel_min"],
        df["vel_max"],
        color=color,
        alpha=0.3,
        label="Min-Max Range",
    )


def plot_velocity(results: list[Result] | Result, name: str, title=None) -> str:
    file = os.path.join(plot_dir(), f"{name}.png")

    if isinstance(results, Result):
        results = [results]

    fig, ax = plt.subplots(figsize=(12, 6))

    for result in results:
        df = result.df()
        desc = result.desc()
        robot_count = len(desc["robots"])
        show_velocities(ax, df, robot_count, label=desc["title"])

    if title is None:
        title = "Velocity over time"

    ax.set_xlabel(r"Time (s)")
    ax.set_ylabel(r"Velocity (m/s)")
    ax.set_title(title)

    if len(results) > 1:
        plt.legend()

    return save_figure(fig, file)


def plot_bytes(results: list[Result] | Result, output_file: str, title=None) -> str:
    if isinstance(results, Result):
        results = [results]

    fig = plt.figure(figsize=(16, 9))

    gs = gridspec.GridSpec(3, 1, figure=fig)

    fig.subplots_adjust(hspace=1)
    byte_ax = fig.add_subplot(gs[0, 0])
    count_ax = fig.add_subplot(gs[1, 0])
    size_ax = fig.add_subplot(gs[2, 0])

    for result in results:
        df = result.df()
        desc = result.desc()

        # Calculate bytes sent per robot
        bytes_per_robot = df["msg-bytes"] / len(desc["robots"])
        df = df.with_columns(bytes_per_robot.alias("bytes-per-robot"))

        # Calculate rolling average
        df = df.with_columns(
            pl.col("bytes-per-robot").rolling_mean(60).alias("bytes-per-robot-smooth")
        )

        byte_ax.scatter(
            df["time"],
            df["bytes-per-robot"],
            label=desc["title"],
            marker="o",
            alpha=0.5,
            s=0.5,
        )
        byte_ax.scatter(
            df["time"],
            df["bytes-per-robot-smooth"],
            label=desc["title"] + " (1s average)",
            marker="o",
            s=1,
        )

        print(
            f"Median bytes sent per robot: {df['bytes-per-robot-smooth'].median():.2f} bytes per second"
        )

        # Plot the number of messages sent
        msgs_per_robot = df["msg-count"] / len(desc["robots"])
        df = df.with_columns(msgs_per_robot.alias("msg-count-per-robot"))
        df = df.with_columns(
            pl.col("msg-count-per-robot")
            .rolling_mean(60)
            .alias("msg-count-per-robot-smooth")
        )

        count_ax.scatter(
            df["time"],
            df["msg-count"],
            label=desc["title"],
            marker="o",
            alpha=0.5,
            s=0.5,
        )
        count_ax.scatter(
            df["time"],
            df["msg-count-per-robot-smooth"],
            label=desc["title"] + " (1s average)",
            marker="o",
            s=1,
        )

        # Plot the size of the messages sent

        # Accumulate the message sizes as a rolling sum
        total_msgs_sent = df["msg-count"].cum_sum().alias("total-msgs-sent")
        total_bytes_sent = df["msg-bytes"].cum_sum().alias("total-bytes-sent")
        avg_msg_size = df["msg-bytes"] / df["msg-count"]
        df = df.with_columns(total_msgs_sent)
        df = df.with_columns(total_bytes_sent)
        df = df.with_columns(avg_msg_size.alias("avg-msg-size"))

        size_ax.scatter(
            df["time"],
            df["avg-msg-size"],
            label="Average Message Size",
            marker="o",
            alpha=0.5,
            s=0.5,
        )

        # print(f"Median message size: {df["avg-msg-size-smooth"].median():.2f} bytes")

    if title is None:
        title = "Data transfer per robot over time"

    byte_ax.set_xlabel("Time (s)")
    byte_ax.set_ylabel("Bytes per Second per Robot")
    byte_ax.set_title(title)

    count_ax.set_xlabel("Time (s)")
    count_ax.set_ylabel("Messages per Second per Robot")
    count_ax.set_title("Messages sent per second per robot")

    size_ax.set_xlabel("Time (s)")
    size_ax.set_ylabel("Average Message Size (bytes)")
    size_ax.set_title("Average message size")

    fig.legend()

    output_file = os.path.join(plot_dir(), output_file)

    return save_figure(fig, output_file)


def plot_spread(
    result: Result | list[Result] | ResultCollection | list[ResultCollection],
    title: str,
) -> str:
    file = os.path.join(plot_dir(), f"{title}.png")

    fig, ax = plt.subplots()

    if not isinstance(result, list):
        result = [result]

    if isinstance(result, list) and all(
        isinstance(item, ResultCollection) for item in result
    ):
        result: list[ResultCollection]
        for collection in result:
            collection = collection.with_spread()
            collection.plot("spread-sd", ax)
    else:
        for res in result:
            df = res.df_with_spread()
            ax.plot(df["time"], df["spread-sd"], label=res.title())

    if len(result) > 1:
        ax.legend()

    return save_figure(fig, file)


def plot_performance(
    result: Result | list[Result], title: str, max=None, force=False
) -> str:
    file = os.path.join(plot_dir(), f"{title}.png")

    # if os.path.exists(file) and not force:
    #     print(f"Plot already exists: '{relpath(file)}'.")
    #     return

    if isinstance(result, Result):
        result = [result]

    fig, ax = plt.subplots()

    for res in result:
        df = res.df()
        ax.scatter(
            df["time"],
            df["step-time"] * 1000,
            marker="o",
            alpha=0.1,
            s=0.5,
            label=res.title(),
        )

    if len(result) > 1:
        ax.legend()

    ax.set_ylim(0, max)

    return save_figure(fig, file)


def plot_paths(
    result: Result,
    title: str,
    segments=1,
    force=False,
    borders=False,
    plot_title: bool = False,
    time_label: bool = True,
) -> list[str]:
    df = result.df()
    desc = result.desc()

    plot_files = []

    for seg in range(segments):
        file = os.path.join(
            plot_dir(), f"{title.replace(' ', '-').lower()}-{seg + 1}-of-{segments}.png"
        )

        if os.path.exists(file) and not force:
            print(f"Plot already exists: '{relpath(file)}'.")
            continue

        fig, ax = plt.subplots()

        t = df["time"]
        end = (len(t) // segments) * (seg + 1)
        try:
            for i, _ in enumerate(desc["robots"]):
                x_col = df[f"robot_{i}/x"]
                y_col = df[f"robot_{i}/y"]

                x_col = x_col[:end]
                y_col = y_col[:end]

                mode_col = None
                if f"robot_{i}/mode" in df.columns:
                    mode_col = df[f"robot_{i}/mode"]
                else:
                    mode_col = pl.Series([0] * len(x_col))

                mode_col = mode_col[:end]

                color = COLORS[i % len(COLORS)]
                cursor = 0

                start = 0
                mode = mode_col[0]

                while cursor < len(mode_col):
                    while cursor < len(mode_col) and mode_col[cursor] == mode:
                        cursor += 1

                    match mode:
                        case 0:
                            ax.plot(
                                x_col[start:cursor],
                                y_col[start:cursor],
                                linestyle="-",
                                color=color,
                                alpha=1.0,
                            )
                        case 1:
                            ax.plot(
                                x_col[start:cursor],
                                y_col[start:cursor],
                                linestyle="-",
                                color=color,
                                alpha=0.3,
                            )
                            ax.plot(
                                x_col[start:cursor],
                                y_col[start:cursor],
                                linestyle="dotted",
                                color=color,
                                alpha=1.0,
                                linewidth=2,
                            )

                    start = cursor
                    if cursor < len(mode_col):
                        mode = mode_col[cursor]

                cross_size = 0.4
                widen = lambda x, size: [x - size, x + size]
                ax.plot(
                    widen(x_col[0], cross_size),
                    widen(y_col[0], cross_size),
                    color=color,
                    linewidth=4,
                    alpha=0.7,
                )
                ax.plot(
                    widen(x_col[0], -cross_size),
                    widen(y_col[0], cross_size),
                    color=color,
                    linewidth=4,
                    alpha=0.7,
                )

                # Draw circle at starting position
                ax.add_patch(
                    patches.Circle(
                        (x_col[-1], y_col[-1]), color=color, radius=0.5, alpha=0.7
                    )
                )

        except pl.exceptions.ColumnNotFoundError as e:
            data_path, desc_path = result.paths()
            print(f"Error using data in files:\n    {data_path}\n    {desc_path}\n")
            print(df)
            print(f"    => {e}")
            exit(1)

        end_time = t[end - 1]

        if time_label:
            # label = patches.Patch(color='none', label=f'Time: {end_time:.0f}s')
            # ax.legend(
            #     handles=[label],
            #     loc='lower right',
            #     frameon=False,
            #     handlelength=0,
            #     handletextpad=0,
            #     borderpad=0.3,
            #     labelspacing=0.2,
            #     fontsize=14,
            # )
            ax.text(
                0.98,
                0.02,  # X, Y in axes fraction (bottom right)
                f"Time: {end_time:.0f}s",
                transform=ax.transAxes,
                ha="right",
                va="bottom",
                fontsize=14,
                bbox=dict(
                    boxstyle="round,pad=0.28", facecolor="#ffffff88", edgecolor="none"
                ),
            )

        plot_file = plot_world(
            fig,
            ax,
            result.world(),
            f"{title} (after {end_time:.0f}s)",
            file,
            borders=borders,
            plot_title=plot_title,
        )
        plot_files.append(plot_file)

    return plot_files

def plot_training(file: str, name: str) -> str:
    print(f"Plotting '{name}' training data from '{file}'")

    stem = os.path.basename(file).replace(".ipc", "").replace(" ", "-").lower()

    plot_file = os.path.join(plot_dir(), f"{stem}.png")

    df = pl.read_ipc(file)

    # Reward Plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

    ax1.plot(
        df["episode"],
        df["reward"],
        label="Reward",
        color=colors.get_color(0),
    )
    ax1.plot(
        df["episode"],
        df["reward"].rolling_mean(10),
        label="Rolling Average",
        color=colors.get_color(1),
    )
    ax1.set_ylabel("Reward")
    ax1.legend()

    # Loss Plot
    ax2.plot(
        df["episode"],
        df["avg_loss"],
        label="Loss",
        color=colors.get_color(2),
    )
    ax2.plot(
        df["episode"],
        df["avg_loss"].rolling_mean(10),
        label="Rolling Average",
        color=colors.get_color(3),
    )
    ax2.set_xlabel("Episode")
    ax2.set_ylabel("Loss")
    ax2.legend()

    # Coverage Plot
    ax3.plot(
        df["episode"],
        df["coverage"] * 100,
        label="Coverage",
        color=colors.get_color(4),
    )
    ax3.plot(
        df["episode"],
        df["coverage"].rolling_mean(10) * 100,
        label="Rolling Average",
        color=colors.get_color(7),
    )
    ax3.set_ylabel(r"Coverage (\%)")
    ax3.set_ylim(0, 80)
    ax3.legend()

    fig.suptitle(f'{name} Training with 10 Episode Rolling Average', fontsize=16)

    fig.align_ylabels()

    save_figure(fig, plot_file)

    return plot_file
