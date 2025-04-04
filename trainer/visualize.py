import polars as pl
import matplotlib.pyplot as plt
import json
import sys

def load_training_data(file_path: str):
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
        return pl.DataFrame(data)
    except Exception as e:
        print(f"Error loading JSON file: {e}")
        return pl.DataFrame()

def live_plot(file_path: str):
    plt.ion()
    fig, axes = plt.subplots(3, 2, figsize=(10, 12))
    fig.subplots_adjust(hspace=0.4, wspace=0.3)

    while True:
        df = load_training_data(file_path)

        if not df.is_empty():
            for ax in axes.flatten():
                ax.clear()

            # Assigning each subplot to a metric
            axes[0, 0].plot(df["episode"], df["reward"], color='tab:blue')
            axes[0, 0].set_title("Reward")
            axes[0, 0].set_xlabel("Episode")

            axes[0, 1].plot(df["episode"], df["avg_loss"], color='tab:red')
            axes[0, 1].set_title("Average Loss")
            axes[0, 1].set_xlabel("Episode")

            axes[1, 0].plot(df["episode"], df["eps"], color='tab:orange')
            axes[1, 0].set_title("Epsilon")
            axes[1, 0].set_xlabel("Episode")

            axes[1, 1].plot(df["episode"], df["coverage"], color='tab:green')
            axes[1, 1].set_title("Coverage")
            axes[1, 1].set_xlabel("Episode")

            axes[2, 0].plot(df["episode"], df["steps"], color='tab:purple')
            axes[2, 0].set_title("Steps per Episode")
            axes[2, 0].set_xlabel("Episode")

            axes[2, 1].plot(df["episode"], df["sim_time"], color='tab:brown')
            axes[2, 1].set_title("Simulation Time")
            axes[2, 1].set_xlabel("Episode")

            plt.pause(1)  # Update interval

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <file_path>")
        sys.exit(1)

    file_path = sys.argv[1]
    live_plot(file_path)
