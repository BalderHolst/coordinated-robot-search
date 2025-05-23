import botplot as bp
import shutil
import os

SEED = 42
DURATION = 400
ROBOTS = 4
BEHAVIORS = [
    ("avoid-obstacles", "Roomba"),
    ("search:gradient", "Gradient"),
    ("search:pathing", "Pure Pathing"),
    ("search:hybrid", "hybrid"),
]

RUNS = 6

WORLD = bp.repo_path("worlds/bitmap/warehouse/warehouse.yaml")
GZ_WORLD = "warehouse"

FIG_DIR = bp.repo_path("report", "figures", "plots")

def main():

    for behavior, behavior_name in BEHAVIORS:

        create_scenario = lambda title: bp.Scenario(
            title=title,
            world=WORLD,
            behavior=behavior,
            duration=DURATION,
            robots=ROBOTS,
            gazebo_world=GZ_WORLD,
        )

        simple_results = []

        bp.seed(SEED)
        for i in range(RUNS):
            print(f"========== Running Simple Sim ({behavior_name}) [{i+1}/{RUNS}] ==========")

            scenario = create_scenario(f"Simple Sim ({behavior_name}) ({i+1} of {RUNS})")

            res = bp.run_sim(scenario)

            bp.plot_paths(res, f"Simple Sim Paths ({behavior_name}) ({i+1} of {RUNS})")

            simple_results.append(res)

        ros_results = []

        bp.seed(SEED)
        for i in range(RUNS):
            print(f"========== Running Gazebo ({behavior_name}) [{i+1}/{RUNS}] ==========")

            scenario = create_scenario(f"Gazebo ({behavior_name}) ({i+1} of {RUNS})")

            res = bp.run_ros(scenario, headless=True)

            bp.plot_paths(res, f"ROS 2 Paths ({behavior_name}) ({i+1} of {RUNS})")

            ros_results.append(res)

        simple_collection = bp.ResultCollection("Simple Simulator", simple_results)
        gazebo_collection = bp.ResultCollection("Gazebo", ros_results)

        src = bp.plot_coverage([simple_collection, gazebo_collection], f"Simulator Coverage over {RUNS} Runs ({behavior_name})")
        dst = os.path.join(FIG_DIR,  f"gazebo_vs_simple_sim_{behavior_name.replace(" ", "_").lower()}.png")

        os.makedirs(FIG_DIR, exist_ok=True)
        shutil.copy(
            src,
            dst
        )

        print(f"Copied plot to '{dst}'")

if __name__ == "__main__":
    main()
