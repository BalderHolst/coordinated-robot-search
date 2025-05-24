from math import inf
import botplot as bp
import shutil
import os
import polars as pl

from polars import DataFrame

SEED = 42
DURATION = 200
ROBOTS = 4
BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:pathing", "Pure Pathing"),
    ("search:hybrid", "hybrid"),
]

RUNS = 6

WORLD = bp.repo_path("worlds/bitmap/depot/depot.yaml")
GZ_WORLD = "depot"

FIG_DIR = bp.repo_path("report", "figures", "plots")


def main():
    # avg_diff_ratios: list[tuple[pl.DataFrame, str]] = []
    avg_diffs: list[tuple[pl.DataFrame, str]] = []
    for behavior, behavior_name in BEHAVIORS:

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
        for i in range(RUNS):
            print(
                f"========== Running Simple Sim ({behavior_name}) [{i + 1}/{RUNS}] =========="
            )

            scenario = create_scenario(
                f"Simple Sim ({behavior_name}) ({i + 1} of {RUNS})"
            )

            res = bp.run_sim(scenario)

            bp.plot_paths(
                res, f"Simple Sim Paths ({behavior_name.title()}) ({i + 1} of {RUNS})"
            )

            simple_results.append(res)

        ros_results = []

        bp.seed(SEED)
        for i in range(RUNS):
            print(
                f"========== Running Gazebo ({behavior_name}) [{i + 1}/{RUNS}] =========="
            )

            scenario = create_scenario(f"Gazebo ({behavior_name}) ({i + 1} of {RUNS})")

            res = bp.run_ros(scenario, headless=True)

            bp.plot_paths(
                res, f"ROS 2 Paths ({behavior_name.title()}) ({i + 1} of {RUNS})"
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
        avg_diffs.append((avg_diff, behavior_name))

        src = bp.plot_coverage(
            [simple_collection, gazebo_collection],
            f"Simulator Coverage over {RUNS} Runs ({behavior_name.title()})",
        )
        dst = os.path.join(
            FIG_DIR,
            f"gazebo_vs_simple_sim_{behavior_name.replace(' ', '_').lower()}.png",
        )

        os.makedirs(FIG_DIR, exist_ok=True)
        shutil.copy(src, dst)

        print(f"Copied plot to '{dst}'")
    # print("Average coverage diff ratios:", avg_diff_ratios)
    # bp.plot_avg_coverage_diff(avg_diff_ratios, "Coverage Diff Ratios")
    bp.plot_avg_coverage_diff(avg_diffs, f"Coverage Diffs of Averages ({RUNS} Runs)")


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
    ratio = compare["coverage"] - base["coverage"]
    df = pl.DataFrame({"time": base["time"], "diff": ratio})
    return df


if __name__ == "__main__":
    main()
