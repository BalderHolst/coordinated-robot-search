import os
import json
import hashlib
import random
from math import sqrt
from dataclasses import dataclass
from typing import Self

import polars as pl
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

from botplot.rust_crate import RustCrate
from botplot.colcon_workspace import ColconWorkspace
import botplot.utils as utils

from botplot.colors import COLORS

mpl.rcParams['axes.prop_cycle'] = mpl.cycler(color=COLORS)

plt.rcParams.update({
    "text.usetex": utils.check_installed("latex"), # Use LaTeX to render all text
    "font.family": "serif",                        # Set font family to serif
    "font.serif": ["Computer Modern Roman"],       # Default LaTeX font
})

mpl.rcParams['axes.edgecolor']   = '#888888'   # gray frame around the plot
mpl.rcParams['xtick.color']      = '#555555'   # gray x ticks
mpl.rcParams['ytick.color']      = '#555555'   # gray y ticks
mpl.rcParams['axes.labelcolor']  = '#444444'   # gray axis labels

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
            raise FileNotFoundError(f"{GIT_DIR} directory not found in any parent directories")
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

botbrain   = RustCrate(repo_path("botbrain"))
simple_sim = RustCrate(repo_path("simple_sim"), dependencies=[botbrain])
trainer    = RustCrate(repo_path("trainer"), dependencies=[simple_sim, botbrain])

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

    def __init__(self, title: str, world: str, behavior: str, duration: int, robots: list[Robot] | int):
        self.title = title
        self.world = world
        self.behavior = behavior
        self.duration = duration

        if isinstance(robots, int):
            robots = place_robots(world, robots)

        self.robots = robots

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
    file: str
    data: dict

    @classmethod
    def from_description_file(cls, file: str) -> Self:

        if file.endswith(".ron"):
            abs_file = os.path.abspath(file)
            hash = hashlib.sha256(abs_file.encode()).hexdigest()
            name = os.path.basename(file)
            json_file = os.path.join(data_dir(), f"{name}-{hash}-desc.json")

            os.makedirs(os.path.dirname(json_file), exist_ok=True)

            if os.path.exists(json_file):
                print(f"Using cached json world description: {relpath(json_file)}")
            else:
                trainer.run([
                    "world-to-json",
                    "--input", file,
                    "--output", json_file,
                    "--force",
                ])
            file = json_file

        with open(file) as f:
            data = json.load(f)

        return cls(file, data["world"])

    def is_obj(self) -> bool:
        return OBJ_LABEL in self.data

    def is_bitmap(self) -> bool:
        return BITMAP_LABEL in self.data

    def desc(self) -> dict:
        if self.is_obj(): return self.data[OBJ_LABEL]
        if self.is_bitmap(): return self.data[BITMAP_LABEL]
        else: raise ValueError(f"World must be either a bitmap or an object world. Found keys: {self.data.keys()}")

    def img(self):

        desc_name = os.path.basename(self.file).replace(".json", "-world-desc.json")
        desc_file = os.path.join(data_dir(), desc_name)

        print(self.file)

        json.dump(self.desc(), open(desc_file, "w"), indent=4)

        img_name = os.path.basename(self.file).replace(".json", ".png")
        img_file = os.path.join(data_dir(), img_name)

        if os.path.exists(img_file):
            print(f"Using cached world image: {relpath(img_file)}")
        else:
            trainer.run([
                "world-to-img",
                "--input", desc_file,
                "--output", img_file,
                "--theme", "grayscale",
                "--force",
            ])

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
        spread   = pl.Series("spread-sd", [0.0] * len(df))
        center_x = pl.Series("center_x",  [0.0] * len(df))
        center_y = pl.Series("center_y",  [0.0] * len(df))

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
    min: pl.DataFrame
    max: pl.DataFrame
    avg: pl.DataFrame

    def __init__(self, name: str, results: list[Result]) -> None:
        self.name = name
        self.results = results

        dfs = [r.df() for r in results]

        cols = dfs[0].columns

        for df in dfs:
            if set(cols) != set(df.columns):
                print("\nError: Result columns do not match")

        self.min = pl.DataFrame()
        self.max = pl.DataFrame()
        self.avg = pl.DataFrame()

        for col in cols:
            runs = [df[col] for df in dfs]
            df_temp = pl.DataFrame({f"s{i}": s for i, s in enumerate(runs)})

            max = df_temp.max_horizontal()
            min = df_temp.min_horizontal()
            avg = df_temp.mean_horizontal()

            self.max = self.max.with_columns((max).alias(col))
            self.min = self.min.with_columns((min).alias(col))
            self.avg = self.avg.with_columns((avg).alias(col))

    def plot(self, col: str, ax, spread=True, color=None, label: bool = True):
        time = self.avg["time"]

        l = None
        if label:
            l = self.name

        line, = ax.plot(time, self.avg[col], linewidth=2, label=l, color=color)
        if spread:
            color = line.get_color()
            ax.fill_between(time, self.min[col], self.max[col], color=color, alpha=0.2)
            ax.plot(time, self.min[col], color=color, alpha=0.8, linewidth=0.5)
            ax.plot(time, self.max[col], color=color, alpha=0.8, linewidth=0.5)
        return line

class SimpleResult(Result):
    """Result from running a scenario with simple_sim."""

class ROSResult(Result):
    """Result from running a scenario with ROS."""


def run_sim(scenario: Scenario | str, headless: bool = True, use_cache=True) -> SimpleResult:

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
    if headless: flags.append("--headless")

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
        print("Error: scenario must be a `Scenario` object or a string. Found: ", type(scenario))
        exit(1)

    print(f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'")

    return SimpleResult(data_file, desc_file)


def run_ros(scenario: Scenario | str, headless: bool = True, use_cache=True) -> ROSResult:

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
    with open(ron_desc_file, 'w') as f:
        f.write(scenario.to_ron())
    simple_sim.run(["world-to-json", ron_desc_file, desc_file])

    print(f"Saved scenario description to '{relpath(desc_file)}'")

    os.makedirs(os.path.dirname(data_file), exist_ok=True)

    flags = ["-o", data_file, "--description", desc_file]
    if headless: flags.append("--headless")

    proc = None

    if isinstance(scenario, Scenario):
        s = scenario.to_ron()
        print(s)

        robots = None
        match scenario.robots:
            case list(): robots = len(scenario.robots)
            case int(): robots = scenario.robots

        proc = ros_ws.launch(
            "multi_robot_control",
            "multi_robot.launch.py",
            behavior="search:pure-pathing",
            n_robots=robots,
            block=False,
            map=scenario.world,
            headless=headless,
            use_rviz=not headless,
        )

        ros_ws.run("multi_robot_control", "data_logger", timeout=scenario.duration, robot_count=robots, output=data_file)

    elif isinstance(scenario, str):
        raise NotImplemented("ROS simulation does not support string scenarios.")
    else:
        print("Error: scenario must be a `Scenario` object or a string. Found: ", type(scenario))
        exit(1)


    print("Killing simulation...")
    proc.kill()
    utils.kill_gazebo()

    print(f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'")

    return ROSResult(data_file, desc_file)

def place_robots_data(world: str | World, n: int) -> dict:
    if isinstance(world, World):
        world = world.file

    flags = []

    proc = trainer.run([
        "place-robots",
        "--world", world,
        "-n", str(n),
        "--seed", str(random.randint(0, 2**64 - 1)),
    ] + flags, capture_output=True, text=True)

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

def plot_world(fig, ax, world: World, title: str, out_file: str, borders: bool, plot_title: bool) -> str:

    if title and plot_title:
        ax.set_title(title, fontsize=16)

    width, height = world.dims()

    xmin, ymin = -width / 2, -height / 2
    xmax, ymax =  width / 2,  height / 2

    if not borders:
        for spine in ax.spines.values():
            spine.set_visible(False)

    plt.margins(0.0)

    # Remove frame ticks
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymax, ymin)
    ax.set_aspect('equal')

    world_img = world.img()
    ax.imshow(world_img, extent=[xmin, xmax, ymin, ymax], origin='lower', zorder=0)

    plt.tight_layout()

    out_file = os.path.join(plot_dir(), f"{title.replace(" ", "-").lower()}.png")

    save_figure(fig, out_file)

    return out_file

def save_figure(fig, output_file: str):
    name = os.path.basename(output_file).replace(" ", "-").lower()
    dir = os.path.dirname(output_file)
    os.makedirs(dir, exist_ok=True)
    output_file = os.path.join(dir, name)
    fig.savefig(output_file, dpi=300, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)
    print(f"Plot saved to '{relpath(output_file)}'")

def plot_coverage(results: Result | list[Result] | ResultCollection | list[ResultCollection], name: str, title=None):

    fig, ax = plt.subplots()

    file = os.path.join(plot_dir(), f"{name}.png")

    if not isinstance(results, list): results = [results]

    if isinstance(results, list) and all(isinstance(item, ResultCollection) for item in results):
        lines = [(c, c.plot("coverage", ax, label=False)) for c in results]
        for c, line in lines:
            c.plot("coverage", ax, color=line.get_color(), spread=False)

    else:

        for result in results:
            df = result.df()
            desc = result.desc()
            ax.plot(df["time"], df["coverage"], label=desc["title"])

    if len(results) > 1: ax.legend()

    if title is None: title = name

    ax.set_xlabel(r"Time (s)")
    ax.set_ylabel(r"Coverage (\%)")
    ax.set_title(title)

    save_figure(fig, file)

def show_velocities(ax, df: pl.DataFrame, robot_count: int, label: str | None = None) -> pl.DataFrame:

    df = df.with_columns([
        pl.col("time").diff().alias("delta_time")
    ])

    for i in range(robot_count):
        df = df.with_columns([
            (pl.col(f"robot_{i}/x").diff().pow(2) + pl.col(f"robot_{i}/y").diff().pow(2)).sqrt().alias(f"robot_{i}/distance"),
        ]).with_columns([
            (pl.col(f"robot_{i}/distance") / pl.col("delta_time")).alias(f"robot_{i}/vel")
        ])

    velocity_cols = [pl.col(f"robot_{i}/vel") for i in range(robot_count)]

    df = df.with_columns([
        pl.min_horizontal(velocity_cols).alias("vel_min"),
        pl.max_horizontal(velocity_cols).alias("vel_max"),
        pl.mean_horizontal(velocity_cols).alias("vel_mean"),
    ])

    line, = ax.plot(df["time"], df["vel_mean"], linewidth=2, label=label)
    color = line.get_color()
    ax.fill_between(df["time"], df["vel_min"], df["vel_max"],
                     color=color, alpha=0.3, label="Min-Max Range")

def plot_velocity(results: list[Result] | Result, name: str, title=None):
    file = os.path.join(plot_dir(), f"{name}.png")

    if isinstance(results, Result): results = [results]

    fig, ax = plt.subplots(figsize=(12, 6))

    for result in results:
        df = result.df()
        desc = result.desc()
        robot_count = len(desc["robots"])
        show_velocities(ax, df, robot_count, label=desc["title"])

    if title is None: title = "Velocity over time"

    ax.set_xlabel(r"Time (s)")
    ax.set_ylabel(r"Velocity (m/s)")
    ax.set_title(title)

    if len(results) > 1: plt.legend()

    save_figure(fig, file)

def plot_bytes(results: list[Result] | Result, output_file: str, title=None):

    if isinstance(results, Result): results = [results]

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
        df = df.with_columns(pl.col("bytes-per-robot").rolling_mean(60).alias("bytes-per-robot-smooth"))

        byte_ax.scatter(df["time"], df["bytes-per-robot"], label=desc["title"], marker="o", alpha=0.5, s=0.5)
        byte_ax.scatter(df["time"], df["bytes-per-robot-smooth"], label=desc["title"] + " (1s average)", marker="o", s=1)

        print(f"Median bytes sent per robot: {df["bytes-per-robot-smooth"].median():.2f} bytes per second")

        # Plot the number of messages sent
        msgs_per_robot = df["msg-count"] / len(desc["robots"])
        df = df.with_columns(msgs_per_robot.alias("msg-count-per-robot"))
        df = df.with_columns(pl.col("msg-count-per-robot").rolling_mean(60).alias("msg-count-per-robot-smooth"))

        count_ax.scatter(df["time"], df["msg-count"], label=desc["title"], marker="o", alpha=0.5, s=0.5)
        count_ax.scatter(df["time"], df["msg-count-per-robot-smooth"], label=desc["title"] + " (1s average)", marker="o", s=1)

        # Plot the size of the messages sent

        # Accumulate the message sizes as a rolling sum
        total_msgs_sent = df["msg-count"].cum_sum().alias("total-msgs-sent")
        total_bytes_sent = df["msg-bytes"].cum_sum().alias("total-bytes-sent")
        avg_msg_size = df["msg-bytes"] / df["msg-count"]
        df = df.with_columns(total_msgs_sent)
        df = df.with_columns(total_bytes_sent)
        df = df.with_columns(avg_msg_size.alias("avg-msg-size"))

        size_ax.scatter(df["time"], df["avg-msg-size"], label="Average Message Size", marker="o", alpha=0.5, s=0.5)


        # print(f"Median message size: {df["avg-msg-size-smooth"].median():.2f} bytes")

    if title is None: title = "Data transfer per robot over time"


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

    save_figure(fig, output_file)

def plot_spread(result: Result | list[Result], title: str, force=False):
    file = os.path.join(plot_dir(), f"{title}.png")

    if os.path.exists(file) and not force:
        print(f"Plot already exists: '{relpath(file)}'.")
        return

    if isinstance(result, Result): result = [result]

    fig, ax = plt.subplots()

    for res in result:
        df = res.df_with_spread()
        ax.plot(df["time"], df["spread-sd"], label=res.title())

    if len(result) > 1: ax.legend()

    save_figure(fig, file)

def plot_performance(result: Result | list[Result], title: str, max=None, force=False):
    file = os.path.join(plot_dir(), f"{title}.png")

    if os.path.exists(file) and not force:
        print(f"Plot already exists: '{relpath(file)}'.")
        return

    if isinstance(result, Result): result = [result]

    fig, ax = plt.subplots()

    for res in result:
        df = res.df()
        ax.scatter(df["time"], df["step-time"] * 1000, marker="o", alpha=0.1, s=0.5, label=res.title())

    if len(result) > 1: ax.legend()

    ax.set_ylim(0, max)

    save_figure(fig, file)

    plt.close(fig)

def plot_paths(result: Result, title: str, segments=1, force=False, borders=False, plot_title: bool = False, time_label: bool=True) -> list[str]:
    df = result.df()
    desc = result.desc()

    plot_files = []

    for seg in range(segments):
        file = os.path.join(plot_dir(), f"{title.replace(" ", "-").lower()}-{seg + 1}-of-{segments}.png")

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
                            ax.plot(x_col[start:cursor], y_col[start:cursor], linestyle="-", color=color, alpha=1.0)
                        case 1:
                            ax.plot(x_col[start:cursor], y_col[start:cursor], linestyle="-", color=color, alpha=0.3)
                            ax.plot(x_col[start:cursor], y_col[start:cursor], linestyle="dotted", color=color, alpha=1.0, linewidth=2)

                    start = cursor
                    if cursor < len(mode_col):
                        mode = mode_col[cursor]

                cross_size = 0.4
                widen = lambda x, size: [x - size, x + size]
                ax.plot(widen(x_col[0],  cross_size), widen(y_col[0], cross_size), color=color, linewidth=4, alpha=0.7)
                ax.plot(widen(x_col[0], -cross_size), widen(y_col[0], cross_size), color=color, linewidth=4, alpha=0.7)

                # Draw circle at starting position
                ax.add_patch(patches.Circle((x_col[-1], y_col[-1]), color=color, radius=0.5, alpha=0.7))

        except pl.exceptions.ColumnNotFoundError as e:
            data_path, desc_path = result.paths()
            print(f"Error using data in files:\n    {data_path}\n    {desc_path}\n")
            print(df)
            print(f"    => {e}")
            exit(1)

        end_time = t[end-1]

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
                0.98, 0.02,               # X, Y in axes fraction (bottom right)
                f'Time: {end_time:.0f}s',
                transform=ax.transAxes,
                ha='right', va='bottom',
                fontsize=14,
                bbox=dict(boxstyle='round,pad=0.28', facecolor='#ffffff88', edgecolor='none')
            )


        plot_file = plot_world(fig, ax, result.world(), f"{title} (after {end_time:.0f}s)", file, borders=borders, plot_title=plot_title)
        plot_files.append(plot_file)

    return plot_files

