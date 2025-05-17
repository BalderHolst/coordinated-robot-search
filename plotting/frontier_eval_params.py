import botplot as bp
import random
import os


def main():
    behaviors = [
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
    worlds = [
        "worlds/bitmap/depot/depot.yaml",
        "worlds/objectmap/medium_obstacles.ron",
    ]

    n = 5
    seed = 0

    for world in worlds:
        world_name = os.path.basename(world).split(".")[0]
        results: list[bp.Result] = []
        for behavior in behaviors:
            random.seed(seed)
            scenario = bp.Scenario(
                title=f"{behavior[len('pathing-') :]}",
                world=world,
                behavior="search:" + behavior,
                duration=200,
                robots=n,
            )

            res = bp.run_sim(scenario)
            results.append(res)

        bp.plot_coverage(
            results,
            f"frontier_eval_params_{world_name}",
            f'Coverage of {n} robots in "{world_name}"',
        )


if __name__ == "__main__":
    main()
