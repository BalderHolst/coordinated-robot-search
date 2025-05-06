import botplot as bp
import random
import os


def main():

    behaviors = [
        "pure-pathing-s33-d33-t33",
        "pure-pathing-s10-d30-t60",
        "pure-pathing-s80-d10-t10",
        "pure-pathing-s10-d80-t10",
        "pure-pathing-s10-d10-t80",
        "pure-pathing-s20-d60-t20",
        "pure-pathing-s10-d45-t45",
        "pure-pathing-s0-d50-t50",
        "pure-pathing-s100-d0-t0",
        "pure-pathing-s0-d100-t0",
        "pure-pathing-s0-d0-t100",
    ]
    worlds = [
        "simple_sim/worlds/bitmap/depot/depot.yaml",
        # "simple_sim/worlds/objectmap/medium_obstacles.ron",
    ]

    n = 1
    seed = 0

    for world in worlds:
        world_name = os.path.basename(world).split(".")[0]
        results: list[bp.Result] = []
        for behavior in behaviors:
            random.seed(seed)
            scenario = bp.Scenario(
                title=f"{behavior[len('pure-pathing-'):]}",
                world=world,
                behavior="search:" + behavior,
                duration=200,
                robots=n,
            )

            res = bp.run_sim(scenario)
            results.append(res)

        bp.plot_coverage(results, f"frontier_eval_params_{world_name}", f"Coverage of {n} robots in \"{world_name}\"")


if __name__ == "__main__":
    main()
