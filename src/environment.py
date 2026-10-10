import numpy as np
import random

class Environment:
    def __init__(self, rows=10, cols=10):
        self.rows = rows
        self.cols = cols

        # Create grid:
        # 0 = free space
        # 1 = obstacle
        self.grid = np.zeros((rows, cols), dtype=int)

    def add_obstacle(self, row, col):
        """Add an obstacle at a given position."""
        if self.is_inside_grid(row, col):
            self.grid[row, col] = 1

    def is_inside_grid(self, row, col):
        """Check if a position is inside the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def is_free(self, row, col):
        """Check if the robot can move to this position."""
        return (
            self.is_inside_grid(row, col)
            and self.grid[row, col] == 0
        )

    def generate_random_obstacles(self, probability=0.2, protected_positions=None):
        """
        Randomly generate obstacles throughout the environment.

        probability = chance that each grid cell becomes an obstacle.
        protected_positions = positions that must remain empty.
        """

        if protected_positions is None:
            protected_positions = []

        for row in range(self.rows):
            for col in range(self.cols):

                if (row, col) in protected_positions:
                    continue

                if random.random() < probability:
                    self.grid[row, col] = 1