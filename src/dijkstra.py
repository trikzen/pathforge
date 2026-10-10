import heapq

from src.astar import get_neighbours, reconstruct_path


def dijkstra(environment, start, goal):
    """
    Find the shortest path from start to goal
    using Dijkstra's algorithm.
    """

    open_set = []

    heapq.heappush(open_set, (0, start))

    came_from = {}

    # Distance from start to each discovered position
    distance = {start: 0}

    # Count how many nodes the algorithm explores
    nodes_explored = 0

    while open_set:

        current_distance, current = heapq.heappop(open_set)

        # Ignore outdated entries in the priority queue
        if current_distance > distance[current]:
            continue

        nodes_explored += 1

        # Goal reached
        if current == goal:
            path = reconstruct_path(came_from, current)

            return path, nodes_explored

        # Explore neighbouring positions
        for neighbour in get_neighbours(environment, current):

            new_distance = distance[current] + 1

            if (
                neighbour not in distance
                or new_distance < distance[neighbour]
            ):
                distance[neighbour] = new_distance
                came_from[neighbour] = current

                heapq.heappush(
                    open_set,
                    (new_distance, neighbour)
                )

    return None, nodes_explored