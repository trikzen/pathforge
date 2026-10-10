import matplotlib.pyplot as plt


def show_environment(
    environment,
    start=None,
    goal=None,
    astar_path=None,
    dijkstra_path=None
):
    """Display the environment with A* and Dijkstra paths."""

    plt.figure(figsize=(8, 8))

    # Draw grid and obstacles
    plt.imshow(environment.grid, cmap="Greys")

    # Draw Dijkstra path first
    if dijkstra_path:
        dijkstra_x = [position[1] for position in dijkstra_path]
        dijkstra_y = [position[0] for position in dijkstra_path]

        plt.plot(
            dijkstra_x,
            dijkstra_y,
            color="orange",
            linewidth=4,
            linestyle="--",
            label="Dijkstra"
        )

    # Draw A* path
    if astar_path:
        astar_x = [position[1] for position in astar_path]
        astar_y = [position[0] for position in astar_path]

        plt.plot(
            astar_x,
            astar_y,
            color="blue",
            linewidth=2,
            label="A*"
        )

    # Draw start
    if start is not None:
        plt.scatter(
            start[1],
            start[0],
            color="green",
            s=140,
            marker="o",
            label="Start",
            zorder=5
        )

    # Draw goal
    if goal is not None:
        plt.scatter(
            goal[1],
            goal[0],
            color="red",
            s=200,
            marker="*",
            label="Goal",
            zorder=5
        )

    plt.xticks(range(environment.cols))
    plt.yticks(range(environment.rows))

    plt.grid(True)
    plt.legend()
    plt.title("A* vs Dijkstra Path Planning")

    plt.show()