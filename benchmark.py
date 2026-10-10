import time

from src.environment import Environment
from src.astar import astar
from src.dijkstra import dijkstra


def run_benchmark(trials=100, grid_size=30, obstacle_probability=0.2):

    astar_times = []
    dijkstra_times = []

    astar_nodes = []
    dijkstra_nodes = []

    astar_path_lengths = []
    dijkstra_path_lengths = []

    successful_trials = 0

    for trial in range(trials):

        # Create a new random environment
        environment = Environment(
            rows=grid_size,
            cols=grid_size
        )

        start = (0, 0)
        goal = (grid_size - 1, grid_size - 1)

        environment.generate_random_obstacles(
            probability=obstacle_probability,
            protected_positions=[start, goal]
        )

        # -------------------------
        # A* benchmark
        # -------------------------

        start_time = time.perf_counter()

        astar_path, astar_explored = astar(
            environment,
            start,
            goal
        )

        astar_runtime = time.perf_counter() - start_time

        # -------------------------
        # Dijkstra benchmark
        # -------------------------

        start_time = time.perf_counter()

        dijkstra_path, dijkstra_explored = dijkstra(
            environment,
            start,
            goal
        )

        dijkstra_runtime = time.perf_counter() - start_time

        # Only compare maps where BOTH algorithms found a path
        if astar_path is not None and dijkstra_path is not None:

            successful_trials += 1

            astar_times.append(astar_runtime)
            dijkstra_times.append(dijkstra_runtime)

            astar_nodes.append(astar_explored)
            dijkstra_nodes.append(dijkstra_explored)

            astar_path_lengths.append(len(astar_path) - 1)
            dijkstra_path_lengths.append(len(dijkstra_path) - 1)

    # Make sure at least one map worked
    if successful_trials == 0:
        print("No successful paths found.")
        return

    # Calculate averages
    average_astar_time = sum(astar_times) / successful_trials
    average_dijkstra_time = sum(dijkstra_times) / successful_trials

    average_astar_nodes = sum(astar_nodes) / successful_trials
    average_dijkstra_nodes = sum(dijkstra_nodes) / successful_trials

    average_astar_length = (
        sum(astar_path_lengths) / successful_trials
    )

    average_dijkstra_length = (
        sum(dijkstra_path_lengths) / successful_trials
    )

    # Display results
    print("\n========== BENCHMARK RESULTS ==========")

    print("Grid size:", f"{grid_size} x {grid_size}")
    print("Trials attempted:", trials)
    print("Successful trials:", successful_trials)

    print("\nA*")
    print("Average runtime:",round(average_astar_time * 1000, 3),"ms")
    print("Average nodes explored:", round(average_astar_nodes, 1))
    print("Average path length:", average_astar_length)

    print("\nDijkstra")
    print("Average runtime:",round(average_dijkstra_time * 1000, 3),"ms")
    print("Average nodes explored:", round(average_dijkstra_nodes, 1))
    print("Average path length:", average_dijkstra_length)


if __name__ == "__main__":
    run_benchmark()