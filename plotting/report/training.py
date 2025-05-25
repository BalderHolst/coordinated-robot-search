import botplot as bp
import shutil
import os

WORLD = bp.repo_path("./worlds/objectmap/pathing_example.ron")

MODELS = [
    {
        "name": "Small Agent",
        "path": bp.repo_path("plotting", "data", "small-search-agent-training.ipc"),
        "duration": 1800,
        "behavior": "tiny-polar-rl",
    },
]

FIG_DIR = bp.repo_path("report", "figures", "rl")

def plot_path(model: dict) -> str:
    scenario = bp.Scenario(
        title=model["name"],
        world=WORLD,
        behavior=model["behavior"],
        duration=model["duration"],
        robots=1
    )

    res = bp.run_sim(scenario)

    return bp.plot_paths(res, title=model["name"], segments=3, borders=True)


def main():
    plots = []

    bp.seed(0)

    for model in MODELS:
        training_plot = bp.plot_training(model["path"], model["name"])
        path_plots     = plot_path(model)
        plots += [training_plot] + path_plots

    print(f"\nSaving figures to {FIG_DIR}")
    os.makedirs(FIG_DIR, exist_ok=True)
    for plot in plots:
        src = plot
        dst = os.path.join(FIG_DIR, os.path.basename(src))
        print(f"    Copying '{bp.relpath(src)}' to '{bp.relpath(dst)}'")
        shutil.copy(src, dst)

if __name__ == "__main__":
    main()
