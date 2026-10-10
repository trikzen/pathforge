import heapq


def heuristic(current, goal):
    """
    Estimate the distance from the current position to the goal
    using Manhattan distance.
    """
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def get_neighbours(environment, position):
    """
    Return all valid positions the robot can move to.
    """

    row, col = position

    # Robot can move up, down, left or right
    possible_moves = [
        (row - 1, col),  # Up
        (row + 1, col),  # Down
        (row, col - 1),  # Left
        (row, col + 1)   # Right
    ]

    neighbours = []

    for new_row, new_col in possible_moves:
        if environment.is_free(new_row, new_col):
            neighbours.append((new_row, new_col))

    return neighbours


def reconstruct_path(came_from, current):
    """
    Work backwards from the goal to reconstruct the final path.
    """

    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()

    return path


def astar(environment, start, goal):
    """
    Find the shortest path from start to goal using A*.
    Returns the path and number of nodes explored.
    """

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    # Keeps track of nodes that have actually been explored
    closed_set = set()

    nodes_explored = 0

    while open_set:

        current_priority, current = heapq.heappop(open_set)

        # Don't process the same node twice
        if current in closed_set:
            continue

        closed_set.add(current)
        nodes_explored += 1

        # Goal reached
        if current == goal:
            path = reconstruct_path(came_from, current)
            return path, nodes_explored

        for neighbour in get_neighbours(environment, current):

            if neighbour in closed_set:
                continue

            tentative_g_score = g_score[current] + 1

            if (
                neighbour not in g_score
                or tentative_g_score < g_score[neighbour]
            ):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g_score

                f_score = (
                    tentative_g_score
                    + heuristic(neighbour, goal)
                )

                heapq.heappush(
                    open_set,
                    (f_score, neighbour)
                )

    return None, nodes_explored