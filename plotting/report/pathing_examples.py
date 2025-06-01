import botplot as bp
import os
import shutil
import random

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

        random.seed(100)

        random_angle = lambda: random.uniform(-3.14, 3.14)

        scenario = bp.Scenario(
            title=name,
            world=bp.repo_path("worlds/objectmap/pathing_example.ron"),
            behavior=behavior,
            duration=DURATION,
            robots=[
                bp.Robot(x=2, y=-6, angle=random_angle()),
                bp.Robot(x=4, y=-6, angle=random_angle()),
                bp.Robot(x=2, y=-8, angle=random_angle()),
                bp.Robot(x=4, y=-8, angle=random_angle()),
            ],
        )

        res = bp.run_sim(scenario)

        plot_files = bp.plot_paths(res, f"{behavior}-paths", segments=3, borders=True)

        bp.plot_velocity(res, f"{behavior}-velocity", f"{behavior} Velocity")

        print(f"Copying plot to report:")

        for src in plot_files:
            dst = os.path.join(PLOT_DIR, "paths", os.path.basename(src))

            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(src, dst)

            print(f"    '{bp.relpath(src)}' copied to '{bp.relpath(dst)}'")

