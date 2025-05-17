import botplot as bp

WORLD = bp.repo_path("worlds/objectmap/pathing_example.ron")

RUNS = 5
DURATION = 600
ROBOTS = 4

BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:hybrid", "Hybrid"),
    ("search:pathing", "Pure Pathing"),
]

def main():

    collections = []

    for behavior, name in BEHAVIORS:
        results: list[bp.Result] = []

        bp.seed(42)
        for i in range(5):
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

    bp.plot_coverage(collections, f"Coverage over {RUNS} runs")


if __name__ == "__main__":
    main()
