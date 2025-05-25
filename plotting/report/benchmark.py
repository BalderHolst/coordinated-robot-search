import botplot as bp
import shutil
import os

RUNS = 10
ROBOTS = [2**i for i in range(6)]

BEHAVIORS = [
    ("Roomba",       "avoid-obstacles"),
    ("Gradient",     "search:gradient"),
    ("Hybrid",       "search:hybrid"),
    ("Pure Pathing", "search:pathing"),
]

WORLDS = [
    ("Depot",     bp.repo_path("worlds/bitmap/depot/depot.yaml"),          800),
    ("Warehouse", bp.repo_path("worlds/bitmap/warehouse/warehouse.yaml"), 1200),
]

REPORT_DIR = bp.repo_path("report", "figures", "plots", "benchmarks")

def index(map: str, robots: int, behavior: int) -> str:
    return f"{map}_{robots}_{behavior}"

class RunStore:
    def __init__(self):
        self.store = {}

    def set(self, map: str, robots: int, behavior: int, item: bp.ResultCollection):
        key = index(map, robots, behavior)
        self.store[key] = item

    def get(self, map: str, robots: int, behavior: int) -> bp.ResultCollection:
        key = index(map, robots, behavior)
        return self.store.get(key)

def run():
    store = RunStore()

    for world_name, world, duration in WORLDS:
        for robots in ROBOTS:
            for behavior_name, behavior in BEHAVIORS:
                results: list[bp.Result] = []

                bp.seed(42)
                for i in range(RUNS):
                    print(f"\n========== [{i+1}/{RUNS}] {behavior_name} with {robots} robots in '{world_name}' ==========")

                    scenario = bp.Scenario(
                        title=f"{behavior_name} run {i+1}",
                        world=world,
                        behavior=behavior,
                        duration=duration,
                        robots=robots,
                    )

                    res = bp.run_sim(scenario)
                    results.append(res)

                collection = bp.ResultCollection(behavior_name, results)
                store.set(world_name, robots, behavior, collection)

    return store

def plot(store: RunStore) -> list[str]:

    plot_files = []

    for world_name, _ in WORLDS:
        for robots in ROBOTS:
            collections = []
            for _, behavior in BEHAVIORS:
                collections.append(store.get(world_name, robots, behavior))

            plot_files.append(
                bp.plot_coverage(collections, f"Coverage over {RUNS} runs with {robots} robots in {world_name}")
            )

    for world_name, _ in WORLDS:
        for behavior_name, behavior in BEHAVIORS:
            collections = []
            for robots in ROBOTS:
                collections.append(store.get(world_name, robots, behavior).with_name(f"{robots} robots"))

            plot_files.append(
                bp.plot_coverage(collections, f"Coverage over {RUNS} runs using {behavior_name} behavior in {world_name}")
            )

    return plot_files

def copy_to_report(plot_files: list[str]):
    for plot_file in plot_files:
        dst = os.path.join(REPORT_DIR, os.path.basename(plot_file))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(plot_file, dst)
        print(f"Plot copied to '{bp.relpath(dst)}'")

def main():
    store = run()
    plots = plot(store)
    copy_to_report(plots)


if __name__ == "__main__":
    main()
