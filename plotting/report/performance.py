import botplot as bp
import polars as pl

SEED = 42
DURATION = 200
ROBOTS = 6
BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:hybrid", "Hybrid"),
    ("search:pathing", "Pure Pathing"),
]

RUNS = 6

WORLD = bp.repo_path("worlds/bitmap/depot/depot.yaml")
USE_CACHE = True

if __name__ == "__main__":
    result_collections: list[bp.ResultCollection] = []
    for behavior, behavior_name in BEHAVIORS:
        results: list[bp.Result] = []

        def create_scenario(title):
            return bp.Scenario(
                title=title,
                world=WORLD,
                behavior=behavior,
                duration=DURATION,
                robots=ROBOTS,
            )

        bp.seed(SEED)
        for i in range(RUNS):
            scenario = create_scenario(
                f"Performance ({behavior_name}, ({i} of {RUNS}))"
            )
            res = bp.run_sim(scenario, no_debug_soup=True, use_cache=USE_CACHE)
            results.append(res)

        collection = bp.ResultCollection(
            f"{behavior_name} - Performance {RUNS} Runs", results, use_cache=USE_CACHE
        )
        result_collections.append(collection)
    bp.plot_boxplot(
        result_collections, "step-time", f"Computation Performance ({RUNS} Runs)"
    )
