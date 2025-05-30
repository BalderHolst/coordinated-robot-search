import botplot as bp
import os
import shutil

BEHAVIORS = [
    ("avoid-obstacles", "Roomba Behavior"),
    ("search:gradient", "Gradient Behavior"),
    ("search:hybrid",   "Hybrid Behavior"),
    ("search:pathing",  "Pathing Behavior"),
]

DURATION = 600;

PLOT_DIR = bp.repo_path("report", "figures", "plots")

if __name__ == "__main__":

    for (behavior, name) in BEHAVIORS:

        bp.seed(2)
        scenario = bp.Scenario(
            title=name,
            world=bp.repo_path("worlds/objectmap/pathing_example.ron"),
            behavior=behavior,
            duration=DURATION,
            robots=4,
        )

        res = bp.run_sim(scenario)

        plot_files = bp.plot_paths(res, f"{behavior}-paths", 3, borders=True)

        bp.plot_velocity(res, f"{behavior}-velocity", f"{behavior} Velocity")

        print(f"Copying plot to report:")

        for src in plot_files:
            dst = os.path.join(PLOT_DIR, "paths", os.path.basename(src))

            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(src, dst)

            print(f"    '{bp.relpath(src)}' copied to '{bp.relpath(dst)}'")

