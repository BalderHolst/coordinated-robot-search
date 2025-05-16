import botplot as bp
import os
import shutil

BEHAVIORS = [
    ("search:gradient", "Gradient Behavior"),
]

DURATION = 900;

PLOT_DIR = bp.repo_path("report", "figures", "plots")

if __name__ == "__main__":

    for (behavior, name) in BEHAVIORS:

        scenario = bp.Scenario(
            title=name,
            world=bp.repo_path("worlds/objectmap/pathing_example.ron"),
            behavior=behavior,
            duration=DURATION,
            robots=[
                bp.Robot(x=0, y=-7),
                bp.Robot(x=2, y=-7),
                bp.Robot(x=0, y=-9),
                bp.Robot(x=2, y=-9),
            ],
        )

        res = bp.run_sim(scenario)

        plot_files = bp.plot_paths(res, f"{behavior}-paths", 3, borders=True, plot_title=False, time_label=True)

        for src in plot_files:
            dst = os.path.join(PLOT_DIR, "gradient-paths", os.path.basename(src))

            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(src, dst)

            print(f"Plot copied to '{bp.relpath(dst)}'")

