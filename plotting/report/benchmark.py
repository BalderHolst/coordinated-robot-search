import botplot as bp
import shutil
import os

from plotting.report.gazebo_vs_simple_sim import WORLD

# WORLD = bp.repo_path("worlds/objectmap/pathing_example.ron")

RUNS = 10
DURATION = 800
ROBOTS = [i+1 for i in range(6)]

BEHAVIORS = [
    ("Roomba",       "avoid-obstacles"),
    ("Gradient",     "search:gradient"),
    ("Hybrid",       "search:hybrid"),
    ("Pure Pathing", "search:pathing"),
]

WORLDS = [
    ("Warehouse", bp.repo_path("worlds/bitmap/warehouse/warehouse.yaml")),
    ("Depot",     bp.repo_path("worlds/bitmap/depot/depot.yaml")),
]

DIR = bp.repo_path("report", "figures", "plots", "benchmarks")

def main():
    plot_files = []

    for world_name, world in WORLDS:

        for robots in ROBOTS:

            collections = []

            for behavior_name, behavior in BEHAVIORS:
                results: list[bp.Result] = []

                bp.seed(42)
                for i in range(RUNS):
                    print(f"\n========== [{i+1}/{RUNS}] {behavior_name} with {robots} robots in '{world_name}' ==========")

                    scenario = bp.Scenario(
                        title=f"{behavior_name} run {i+1}",
                        world=world,
                        behavior=behavior,
                        duration=DURATION,
                        robots=robots,
                    )

                    res = bp.run_sim(scenario)
                    results.append(res)

                collections.append(bp.ResultCollection(behavior_name, results))


            plot_files.append(
                bp.plot_coverage(collections, f"Coverage over {RUNS} runs with {robots} robots in {world_name}")
            )

            bp.plot_spread(collections, f"Spread over {RUNS} runs with {robots} robots in {world_name}")

        for plot_file in plot_files:
            dst = os.path.join(DIR, os.path.basename(plot_file))
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(plot_file, dst)
            print(f"Plot copied to '{bp.relpath(dst)}'")



if __name__ == "__main__":
    main()
