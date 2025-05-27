import botplot as bp
import botplot.utils as utils
import time
import os

RUNS = 5
DURATION = 600

def main():
    bp.seed(42)

    simple_sim_times = []
    gazebo_sim_times = []

    for _ in range(RUNS):
        scenario = bp.Scenario(
            title="Simulator Benchmark",
            world=bp.repo_path("worlds", "bitmap", "warehouse", "warehouse.yaml"),
            gazebo_world="warehouse",
            behavior="avoid-obstacles",
            duration=DURATION,
            robots=4,
        )

        stem = scenario.title.replace(" ", "-").lower()
        hash = scenario.hash()

        simple_sim_time_file = bp.data_dir(f"{stem}-{hash}-simple.txt")
        gazebo_sim_time_file = bp.data_dir(f"{stem}-{hash}-gazebo.txt")

        if os.path.exists(simple_sim_time_file):
            print(f"Using cached simple sim time: {simple_sim_time_file}")
        else:
            start = time.time()
            bp.run_sim(scenario)
            elapsed = time.time() - start

            with open(simple_sim_time_file, 'w') as f:
                f.write(str(elapsed))

        if os.path.exists(gazebo_sim_time_file):
            print(f"Using cached Gazebo sim time: {gazebo_sim_time_file}")
        else:
            utils.kill_gazebo()
            time.sleep(3)
            start = time.time()
            bp.run_ros(scenario)
            elapsed = time.time() - start
            with open(gazebo_sim_time_file, 'w') as f:
                f.write(str(elapsed))

        with open(simple_sim_time_file) as f:
            simple_sim_times.append(float(f.readline().strip()))

        with open(gazebo_sim_time_file) as f:
            gazebo_sim_times.append(float(f.readline().strip()))

        print(simple_sim_times)
        print(gazebo_sim_times)

    print(f"\nSimulator Performance Report - {RUNS} runs for {DURATION} seconds")
    print(f"    Simple Sim: {sum(simple_sim_times) / len(simple_sim_times)}")
    print(f"    Gazebo    : {sum(gazebo_sim_times) / len(gazebo_sim_times)}")



if __name__ == "__main__":
    main()
