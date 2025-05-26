import botplot as bp

BIASES = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
BEHAVIORS = [(b, f"search:gradient-f{b:.1f}") for b in BIASES]
SEED = 42
RUNS = 6

def create_scenario(title, behavior):
    return bp.Scenario(
        title=title,
        world=bp.repo_path("worlds/bitmap/depot/depot.yaml"),
        behavior=behavior,
        duration=400,
        robots=4,
        gazebo_world="depot",
    )

def main():

    runs = []

    # Run Roomba
    results = []
    bp.seed(SEED)
    for i in range(RUNS):
        scenario = create_scenario("Roomba", "avoid-obstacles")
        res = bp.run_sim(scenario)
        results.append(res)
    runs.append(bp.ResultCollection("Roomba", results, style={
        "linestyle": "--",
        "linewidth": 2.5,
        "alpha": 1.0,
        "color": bp.colors.get_color(13)
    }))

    for bias, behavior in BEHAVIORS:
        print(f"========== Running Forward Bias ({behavior}) ==========")

        results = []

        bp.seed(SEED)
        for i in range(RUNS):
            title = f"Forward Bias: {bias:.1f} (run {i+1})"
            scenario = create_scenario(title, behavior)
            res = bp.run_sim(scenario)
            results.append(res)

        title = f"Forward Bias: {bias:.1f}"
        c = bp.ResultCollection(title, results)
        runs.append(c)


    bp.plot_coverage(runs, "Forward Bias Coverage")



if __name__ == "__main__":
    main()
