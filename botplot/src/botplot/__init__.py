import os
import subprocess
import json
import hashlib
from math import sqrt
from dataclasses import dataclass

import polars as pl
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

from botplot.crate import RustCrate

from botplot.colors import COLORS

mpl.rcParams['axes.prop_cycle'] = mpl.cycler(color=COLORS)

plt.rcParams.update({
    "text.usetex": True,        # Use LaTeX to render all text
    "font.family": "serif",     # Set font family to serif
    "font.serif": ["Computer Modern Roman"],  # Default LaTeX font
})

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
    return os.path.join(root_dir(), *path)

def data_dir(): return repo_path("data")
def plot_dir(): return repo_path("plots")

simulator = RustCrate(repo_path("simple_sim"))
trainer   = RustCrate(repo_path("trainer"))

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
    file: str
    data: dict

    def is_obj(self) -> bool:
        return OBJ_LABEL in self.data

    def is_bitmap(self) -> bool:
        return BITMAP_LABEL in self.data

    def desc(self) -> dict:
        if self.is_obj(): return self.data[OBJ_LABEL]
        if self.is_bitmap(): return self.data[BITMAP_LABEL]
        else: raise ValueError(f"World must be either a bitmap or an object world. Found keys: {self.data.keys()}")

    def img(self):

        if self.is_obj():
            desc_name = os.path.basename(self.file).replace(".json", "-desc.json")
            desc_file = os.path.join(data_dir(), desc_name)

            json.dump(self.data[OBJ_LABEL], open(desc_file, "w"), indent=4)

            img_name = os.path.basename(self.file).replace(".json", ".png")
            img_file = os.path.join(data_dir(), img_name)

            if os.path.exists(img_file):
                print(f"Using cached world image: {relpath(img_file)}")
            else:
                trainer.run([
                    "world-to-img",
                    "--input", desc_file,
                    "--output", os.path.join(data_dir(), img_file),
                    "--theme", "grayscale",
                    "--force",
                ])

            return plt.imread(img_file)


        pass

    def dims(self) -> tuple[float, float]:
        desc = self.desc()
        if self.is_obj():
            return desc["width"], desc["height"]
        if self.is_bitmap():
            raise NotImplemented("Bitmap world not implemented yet.")

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

    def df_with_spread(self) -> pl.DataFrame:
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

        return df

    def desc(self) -> dict:
        with open(self.description_file) as f:
            return json.load(f)

    def paths(self) -> tuple[str, str]:
        """Returns the releative paths to the data and description files."""
        data_path = os.path.relpath(self.dataframe_file, os.path.curdir)
        desc_path = os.path.relpath(self.description_file, os.path.curdir)
        return data_path, desc_path

    def world(self) -> World:
        return World(self.description_file, self.desc()["world"])


def run_sim(scenario: Scenario | str, headless: bool = True, use_cache=True, verbose=False) -> Result:

    match scenario:
        case Scenario():
            name = scenario.title
        case str():
            name = scenario.replace("/", "-")
        case _:
            raise ValueError("Scenario must be either a string or a Scenario object.")

    hash = hashlib.sha256((name + str(scenario)).encode()).hexdigest()

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
        simulator.run(["scenario", "-"] + flags, input=s, text=True)
    elif isinstance(scenario, str):
        if not os.path.exists(scenario):
            print(f"Error: Scenario file '{scenario}' does not exist.")
            exit(1)
        simulator.run(["scenario", scenario] + flags)
    else:
        print("Error: scenario must be a `Scenario` object or a string. Found: ", type(scenario))
        exit(1)

    print(f"Simulation output saved to './{os.path.relpath(data_file, os.path.curdir)}'")

    return Result(data_file, desc_file)

def world_plot(fig, ax, result: Result, title: str, out_file: str):

    if title:
        ax.set_title(title, fontsize=16)

    world = result.world()

    width, height = world.dims()

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

    world_img = world.img()
    ax.imshow(world_img, extent=[xmin, xmax, ymin, ymax], origin='lower', zorder=0)

    plt.tight_layout()

    # Make sure directory exists
    os.makedirs(os.path.dirname(out_file), exist_ok=True)

    fig.savefig(out_file, dpi=300, bbox_inches='tight', pad_inches=0.2)

def save_figure(fig, output_file: str):
    fig.savefig(output_file, dpi=300, bbox_inches='tight', pad_inches=0.2)
    plt.close(fig)
    print(f"Plot saved to '{relpath(output_file)}'")

def plot_coverage(results: list[Result] | Result, output_file: str, title=None):

    if isinstance(results, Result): results = [results]

    for result in results:
        df = result.df()
        desc = result.desc()
        plt.plot(df["time"], df["coverage"], label=desc["title"])

    if title is None: title = "Coverage over time"

    plt.xlabel(r"Time (s)")
    plt.ylabel(r"Coverage (\%)")
    plt.title(title)
    plt.legend()

    output_file = os.path.join(plot_dir(), output_file)

    plt.savefig(output_file, dpi=300, bbox_inches='tight', pad_inches=0.2)
    plt.close()

    print(f"Plot saved to '{relpath(output_file)}'")

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

def plot_spread(result: Result, title: str):
    df = result.df_with_spread()

    fig, ax = plt.subplots()

    ax.plot(df["time"], df["spread-sd"], label="Spread")

    file = os.path.join(plot_dir(), f"{title}.png")

    save_figure(fig, file)

def plot_performance(result: Result, title: str, max=None):
    df = result.df()

    fig, ax = plt.subplots()

    ax.plot(df["time"], df["step-time"] * 1000, label="Average Step Time [ms]")

    ax.set_ylim(0, max)

    file = os.path.join(plot_dir(), f"{title}.png")

    save_figure(fig, file)

def plot_paths(result: Result, title: str, segments=1):
    df = result.df()
    desc = result.desc()

    for seg in range(segments):
        fig, ax = plt.subplots()

        t = df["time"]
        end = (len(t) // segments) * (seg + 1)
        try:
            for i, _ in enumerate(desc["robots"]):

                x_col = df[f"robot_{i}/x"]
                y_col = df[f"robot_{i}/y"]
                mode_col = df[f"robot_{i}/mode"]

                x_col = x_col[:end]
                y_col = y_col[:end]
                mode_col = mode_col[:end]

                color = None
                cursor = 0

                start = 0
                mode = mode_col[0]

                while cursor < len(mode_col):
                    while cursor < len(mode_col) and mode_col[cursor] == mode:
                        cursor += 1
                    linestyle = '-' if mode == 0 else 'dotted'
                    # print(f"{t_col[start]:.0f}s - {t_col[cursor-1]:.0f}s => mode: {mode}")
                    line, = ax.plot(x_col[start:cursor], y_col[start:cursor], linestyle=linestyle, color=color, alpha=0.8)
                    if color is None: color = line.get_color()
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

        file = os.path.join(plot_dir(), f"{title.replace(" ", "-").lower()}-{seg + 1}-of-{segments}.png")

        world_plot(fig, ax, result, f"{title} (after {t[end-1]:.0f}s)", file)


        print(f"Plot saved to '{relpath(file)}'")
