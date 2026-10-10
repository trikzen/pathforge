from src.environment import Environment
from src.astar import astar
from src.dijkstra import dijkstra
from src.visualise import show_environment


def main():

    environment = Environment(rows=20, cols=20)

    start = (1, 1)
    goal = (18, 18)

    environment.generate_random_obstacles(
        probability=0.2,
        protected_positions=[start, goal]
    )

    # Run A*
    astar_path, astar_nodes = astar(
        environment,
        start,
        goal
    )

    # Run Dijkstra
    dijkstra_path, dijkstra_nodes = dijkstra(
        environment,
        start,
        goal
    )

    print("\n--- A* ---")

    if astar_path:
        print("Path length:", len(astar_path) - 1)
        print("Nodes explored:", astar_nodes)
    else:
        print("No path found.")

    print("\n--- Dijkstra ---")

    if dijkstra_path:
        print("Path length:", len(dijkstra_path) - 1)
        print("Nodes explored:", dijkstra_nodes)
    else:
        print("No path found.")

    show_environment(
        environment,
        start=start,
        goal=goal,
        astar_path=astar_path,
        dijkstra_path=dijkstra_path
    )


if __name__ == "__main__":
    main()