import polars as pl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
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
    fig = plt.figure(figsize=(12, 16))

    fig.subplots_adjust(hspace=1)

    gs = gridspec.GridSpec(4, 2, figure=fig)

    axes = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(8)]

    try:
        while True:
            df = load_training_data(file_path)

            if not df.is_empty():
                for ax in axes:
                    ax.clear()

                # Standard plots
                axes[0].plot(df["episode"], df["reward"], color='tab:blue')
                axes[0].set_title("Reward")
                axes[0].set_xlabel("Episode")

                axes[1].plot(df["episode"], df["avg_loss"], color='tab:red')
                axes[1].set_title("Average Loss")
                axes[1].set_xlabel("Episode")

                axes[2].plot(df["episode"], df["eps"], color='tab:orange')
                axes[2].set_title("Epsilon")
                axes[2].set_xlabel("Episode")

                axes[3].plot(df["episode"], df["coverage"]*100, color='tab:green')
                axes[3].set_title(r"Coverage (%)")
                axes[3].set_xlabel("Episode")
                axes[3].set_ylim(0, 100)

                axes[4].plot(df["episode"], df["steps"], color='tab:purple')
                axes[4].set_title("Steps per Episode")
                axes[4].set_xlabel("Episode")

                axes[5].plot(df["episode"], df["sim_time"], color='tab:brown')
                axes[5].set_title("Simulation Time")
                axes[5].set_xlabel("Episode")

                n = df["robots"].max()
                axes[6].scatter(df["episode"], df["robots"], color='tab:pink', marker='o', s=50)
                axes[6].set_title("Robots")
                axes[6].set_xlabel("Episode")
                axes[6].set_ylim(0, n + 1)
                axes[6].set_yticks(range(0, n + 1))
                # Horizontal grid lines
                axes[6].yaxis.grid(True, linestyle='--', alpha=0.7)

                # Stacked area chart: Action percentage distribution
                episodes = df["episode"].to_list()
                num_episodes = len(df)

                # Determine the max number of actions seen
                max_actions = max(len(actions) for actions in df["actions"])

                # Build 2D list: actions_percentages[action_id][episode_index]
                actions_percentages = [[0] * num_episodes for _ in range(max_actions)]

                for i, action_counts in enumerate(df["actions"]):
                    total = sum(action_counts)
                    if total == 0:
                        continue
                    for action_id, count in enumerate(action_counts):
                        actions_percentages[action_id][i] = (count / total) * 100

                colors = plt.cm.tab20.colors  # Up to 20 distinct colors

                axes[7].stackplot(
                    episodes,
                    actions_percentages,
                    labels=[f"Action {i}" for i in range(max_actions)],
                    colors=colors[:max_actions],
                    alpha=0.8
                )

                axes[7].set_title("Action Distribution per Episode (Stacked Area)")
                axes[7].set_xlabel("Episode")
                axes[7].set_ylim(0, 100)

            plt.pause(1)
    except KeyboardInterrupt:
        print("Stopped live plotting.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <file_path>")
        sys.exit(1)

    file_path = sys.argv[1]
    live_plot(file_path)
