import botplot as bp
import shutil
import os

WORLD = bp.repo_path("worlds/objectmap/pathing_example.ron")

RUNS = 10
DURATION = 800
ROBOTS = [i+1 for i in range(6)]

BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:hybrid", "Hybrid"),
    ("search:pathing", "Pure Pathing"),
]

DIR = bp.repo_path("report", "figures", "plots", "benchmarks")

def main():
    plot_files = []

    for robots in ROBOTS:

        collections = []

        for behavior, name in BEHAVIORS:
            results: list[bp.Result] = []

            bp.seed(42)
            for i in range(RUNS):
                print(f"\n========== [{i+1}/{RUNS}] {name} with {robots} robots ==========")

                scenario = bp.Scenario(
                    title=f"{name} run {i+1}",
                    world=WORLD,
                    behavior=behavior,
                    duration=DURATION,
                    robots=robots,
                )

                res = bp.run_sim(scenario, f"multiple_runs/run-{i+1}")
                results.append(res)

            collections.append(bp.ResultCollection(f"{name} with {robots} robots", results))


        plot_files.append(
            bp.plot_coverage(collections, f"Coverage over {RUNS} runs")
        )

        plot_files.append(
            bp.plot_spread(collections, f"Spread over {RUNS} runs")
        )

    for plot_file in plot_files:
        dst = os.path.join(DIR, os.path.basename(plot_file))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(plot_file, dst)
        print(f"Plot copied to '{bp.relpath(dst)}'")



if __name__ == "__main__":
    main()
