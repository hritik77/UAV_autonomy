import numpy as np

class Simulator2D:
    def __init__(self, start, goal, dt):
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)

        self.dt=dt

        self.measured_pos = self.pos.copy()

        #Logging Path
        self.path_x = [self.pos[0]]
        self.path_y = [self.pos[1]]

    def distance_to_goal(self):
        return np.linalg.norm(self.goal - self.pos)

    def step(self,vel_noisy):
        self.pos += vel_noisy*self.dt
        
        self.path_x.append(self.pos[0])
        self.path_y.append(self.pos[1])

        speed = np.linalg.norm(vel_noisy)
        return self.pos, speed
