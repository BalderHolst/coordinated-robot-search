import botplot as bp
import shutil
import os

WORLD = bp.repo_path("worlds/objectmap/pathing_example.ron")

RUNS = 10
DURATION = 800
ROBOTS = 4

BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:hybrid", "Hybrid"),
    ("search:pathing", "Pure Pathing"),
]

DIR = bp.repo_path("report", "figures", "plots", "benchmarks")

def main():

    collections = []

    for behavior, name in BEHAVIORS:
        results: list[bp.Result] = []

        bp.seed(42)
        for i in range(RUNS):
            print(f"{name} run {i+1}/{RUNS}")

            scenario = bp.Scenario(
                title=f"{name} run {i+1}",
                world=WORLD,
                behavior=behavior,
                duration=DURATION,
                robots=ROBOTS,
            )

            res = bp.run_sim(scenario, f"multiple_runs/run-{i+1}")
            results.append(res)

        collections.append(bp.ResultCollection(name, results))

    plot_file = bp.plot_coverage(collections, f"Coverage over {RUNS} runs")

    dst = os.path.join(DIR, os.path.basename(plot_file))

    os.makedirs(os.path.dirname(dst), exist_ok=True)

    shutil.copyfile(
        plot_file,
        dst,
    )

    print(f"Plot copied to '{dst}'")




if __name__ == "__main__":
    main()
