import botplot as bp
import botplot.colors as colors
import matplotlib.pyplot as plt
import random

def main():
    random.seed(2)

    path = bp.repo_path("trainer/worlds/world_0.ron")
    # path = bp.repo_path("simple_sim/worlds/bitmap/depot/depot.yaml")
    world = bp.World.from_description_file(path)


    fig, ax = plt.subplots()

    for n in range(1, 12):
        print(n)

        placement = bp.place_robots_data(path, n)
        robots = placement["robots"]

        # Draw spawn square
        center_x = placement["square_center"]["x"]
        center_y = placement["square_center"]["y"]
        size = placement["square_size"]
        square = plt.Rectangle(
            (center_x - size / 2, center_y - size / 2),
            size,
            size,
            color="gray",
            fill=False,
            alpha=0.2,
        )
        ax.add_artist(square)

        # Draw a circle at robot positions
        for i, robot in enumerate(robots):
            circle = plt.Circle((robot.x, robot.y), 0.35, color=colors.get_color(i), alpha=1, linewidth=0)
            ax.add_artist(circle)


    bp.plot_world(fig, ax, world, "Robot Placement", "placement")


if __name__ == "__main__":
    main()
