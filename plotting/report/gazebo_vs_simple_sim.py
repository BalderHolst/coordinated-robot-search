from math import inf
import botplot as bp
import shutil
import os
import polars as pl
import matplotlib.pyplot as plt

SEED = 42
DURATION = 200
ROBOTS = 4
BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:pathing", "Pure Pathing"),
    ("search:hybrid", "hybrid"),
]

RUNS = 8

WORLD = bp.repo_path("worlds/bitmap/depot/depot.yaml")
GZ_WORLD = "depot"

FIG_DIR = bp.repo_path("report", "figures", "plots", "consistency")


def main():

    fig, axes = plt.subplots(
        nrows=RUNS,
        ncols=len(BEHAVIORS) * 2,
        figsize=(32, 16),
    )

    cov_fig, cov_axes = plt.subplots(
        nrows=2,
        ncols=2,
        figsize=(7, 6),
        sharex=True,
        sharey=True,
    )

    # avg_diff_ratios: list[tuple[pl.DataFrame, str]] = []
    avg_diffs: list[tuple[pl.DataFrame, str]] = []
    for i, (behavior, behavior_name) in enumerate(BEHAVIORS):

        def create_scenario(title):
            return bp.Scenario(
                title=title,
                world=WORLD,
                behavior=behavior,
                duration=DURATION,
                robots=ROBOTS,
                gazebo_world=GZ_WORLD,
            )

        simple_results = []

        bp.seed(SEED)
        for j in range(RUNS):
            print(
                f"========== Running Simple Sim ({behavior_name}) [{j + 1}/{RUNS}] =========="
            )

            scenario = create_scenario(
                f"Simple Sim ({behavior_name}) [{j + 1}]"
            )

            res = bp.run_sim(scenario)

            title = f"Simple Sim Paths ({behavior_name.title()}) ({j + 1} of {RUNS})"
            ax = axes[j, 2 * i]
            ax.set_title(title)
            bp.plot_paths(
                res, title, time_label=False, ax=ax
            )

            simple_results.append(res)

        ros_results = []

        bp.seed(SEED)
        for j in range(RUNS):
            print(
                f"========== Running Gazebo ({behavior_name}) [{j + 1}/{RUNS}] =========="
            )

            scenario = create_scenario(f"Gazebo ({behavior_name}) ({j + 1} of {RUNS})")

            res = bp.run_ros(scenario, headless=False)

            title = f"ROS 2 Paths ({behavior_name.title()}) ({j + 1} of {RUNS})"
            ax = axes[j, 2 * i + 1]
            ax.set_title(title)
            bp.plot_paths(
                res, title, time_label=False, ax=ax
            )

            ros_results.append(res)

        # Find the reference time
        reference_time = bp.interpolated_resample_reference_time(
            ros_results + simple_results
        )
        # Fill in the reference time for the other results
        simple_results = bp.interpolated_resample_coverage(
            simple_results, reference_time
        )
        # Fill in the reference time for the other results
        ros_results = bp.interpolated_resample_coverage(ros_results, reference_time)
        simple_collection = bp.ResultCollection("Simple Simulator", simple_results)
        gazebo_collection = bp.ResultCollection("Gazebo", ros_results)

        # avg_diff_ratio = diff_radio(
        #     gazebo_collection.avg_df(), simple_collection.avg_df()
        # )
        # avg_diff_ratios.append((avg_diff_ratio, behavior_name))

        avg_diff = diff(gazebo_collection.avg_df(), simple_collection.avg_df())
        avg_diffs.append((avg_diff, behavior_name.title()))

        bp.plot_coverage(
            [simple_collection, gazebo_collection],
            behavior_name.title(),
            ax=cov_axes[i // 2, i % 2],
        )

    cov_axes[0, 0].set_xlabel("")
    cov_axes[0, 1].set_xlabel("")
    cov_axes[0, 1].set_ylabel("")
    cov_axes[1, 1].set_ylabel("")

    cov_fig.suptitle(f"Simulator Coverage Comparison over {RUNS} Runs", fontsize=16)
    src = bp.save_figure(cov_fig, bp.plot_dir("gazebo_vs_simple_sim_coverage.png"))
    dst = os.path.join(FIG_DIR, "gazebo_vs_simple_sim_coverage.png")
    os.makedirs(FIG_DIR, exist_ok=True)
    shutil.copy(src, dst)

    print(f"Copied plot to '{dst}'")

    src = bp.plot_avg_coverage_diff(avg_diffs, f"Difference in Coverage between Simulators ({RUNS} Runs)")
    os.makedirs(FIG_DIR, exist_ok=True)
    dst = os.path.join(
        FIG_DIR,
        f"sim_coverage_diff.png",
    )
    shutil.copy(src, dst)

    bp.save_figure(fig, bp.plot_dir("gazebo_vs_simple_sim_paths.png"))


def diff_radio(base: pl.DataFrame, compare: pl.DataFrame) -> pl.DataFrame:
    ratio = (base["coverage"] - compare["coverage"]) / compare["coverage"]
    df = pl.DataFrame({"time": base["time"], "diff": ratio})
    df = df.with_columns(
        pl.col("diff").map_elements(
            lambda x: 0 if x == inf or x == -inf else x, return_dtype=pl.Float64
        )
    )
    return df


def diff(base: pl.DataFrame, compare: pl.DataFrame) -> pl.DataFrame:
    ratio = (compare["coverage"] - base["coverage"]) * 100
    df = pl.DataFrame({"time": base["time"], "diff": ratio})
    return df


if __name__ == "__main__":
    main()
