import botplot as bp
import random
import os


BEHAVIORS = [
    "pathing-s33-d33-t33",
    "pathing-s20-d60-t20",
    "pathing-s10-d45-t45",
    "pathing-s10-d30-t60",
    "pathing-s0-d50-t50",
    "pathing-s80-d10-t10",
    "pathing-s10-d80-t10",
    "pathing-s10-d10-t80",
    "pathing-s100-d0-t0",
    "pathing-s0-d100-t0",
    "pathing-s0-d0-t100",
]
WORLDS = [
    "worlds/objectmap/medium_obstacles.ron",
    "worlds/bitmap/depot/depot.yaml",
    "worlds/bitmap/warehouse/warehouse.yaml",
]

ROBOTS = 5
SEED = 42
RUNS = 5

USE_CACHE = True


def main():
    for world in WORLDS:
        world_name = os.path.basename(world).split(".")[0]
        result_collection: list[bp.ResultCollection] = []
        for behavior in BEHAVIORS:
            results: list[bp.Result] = []
            bp.seed(SEED)
            for i in range(RUNS):
                scenario = bp.Scenario(
                    title=f"{behavior[len('pathing-') :]} {i}",
                    world=world,
                    behavior="search:" + behavior,
                    duration=200,
                    robots=ROBOTS,
                )
                res = bp.run_sim(scenario, use_cache=USE_CACHE)
                results.append(res)
            result_collection.append(
                bp.ResultCollection(
                    f"{behavior[len('pathing-') :]}", results, use_cache=USE_CACHE
                )
            )

        bp.plot_coverage(
            result_collection,
            f"frontier_eval_params_{world_name}",
            f'Coverage of {ROBOTS} robots in "{world_name}" ({RUNS} Runs)',
            spread=False,
            figsize=(6, 4),
            y_limit=None,
        )


if __name__ == "__main__":
    main()
