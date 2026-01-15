import numpy as np

class Simulator2D:
    def __init__(self, start, goal, velocity):
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.vel = np.array(velocity, dtype=float)

        self.path_x = []
        self.path_y = []

    def step(self):
        self.pos += self.vel
        self.path_x.append(self.pos[0])
        self.path_y.append(self.pos[1])

        return self.pos, self.path_x, self.path_y
