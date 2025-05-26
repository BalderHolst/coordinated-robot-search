import botplot as bp
import polars as pl

SEED = 42
DURATION = 200
ROBOTS = 6
BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:pathing", "Pure Pathing"),
    ("search:hybrid", "Hybrid"),
]

RUNS = 6

WORLD = bp.repo_path("worlds/bitmap/depot/depot.yaml")
USE_CACHE = False

if __name__ == "__main__":
    avg_performance: list[tuple[pl.DataFrame, str]] = []
    best_performance: list[tuple[pl.DataFrame, str]] = []
    worst_performance: list[tuple[pl.DataFrame, str]] = []
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
            f"Performance ({behavior_name}, {RUNS} Runs)", results, use_cache=USE_CACHE
        )
        avg_performance.append((collection.avg_df(), behavior_name))
        best_performance.append((collection.min_df(), behavior_name))
        worst_performance.append((collection.max_df(), behavior_name))

    print("Average Performance:")
    for df, behavior in avg_performance:
        print(f"    {behavior}: {(df['step-time'] * 1000).mean()} ms (mean)")
        print(f"    {behavior}: {(df['step-time'] * 1000).min()} ms (min)")
        print(f"    {behavior}: {(df['step-time'] * 1000).max()} ms (max)")

    print("Best Performance:")
    for df, behavior in best_performance:
        print(f"    {behavior}: {(df['step-time'] * 1000).mean()} ms (mean)")
        print(f"    {behavior}: {(df['step-time'] * 1000).min()} ms (min)")
        print(f"    {behavior}: {(df['step-time'] * 1000).max()} ms (max)")

    print("Worst Performance:")
    for df, behavior in worst_performance:
        print(f"    {behavior}: {(df['step-time'] * 1000).mean()} ms (mean)")
        print(f"    {behavior}: {(df['step-time'] * 1000).min()} ms (min)")
        print(f"    {behavior}: {(df['step-time'] * 1000).max()} ms (max)")

    bp.plot_performance_df(avg_performance, "Mean Performance")
    bp.plot_performance_df(best_performance, "Best Performance")
    bp.plot_performance_df(worst_performance, "Worst Performance")
