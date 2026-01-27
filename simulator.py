import numpy as np

class Simulator2D:
    def __init__(self, start, goal, max_speed,dt,mu,sigma):
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.max_speed = max_speed
        self.dt=dt


        self.mu = mu
        self.sigma = sigma

        self.path_x = [self.pos[0]]
        self.path_y = [self.pos[1]]

    def distance_to_goal(self):
        return np.linalg.norm(self.goal - self.pos)
    
    def gaussian_speed(self, distance):
        return self.max_speed * np.exp(-((distance - self.mu) ** 2) / (2 * self.sigma ** 2))
    
    def velocity_vector(self):
        direction = self.goal - self.pos
        distance = np.linalg.norm(direction)
        if distance == 0:
            return np.zeros(2)
        unit_direction = direction / distance
        speed = self.gaussian_speed(distance)
        max_allowed_speed = distance / self.dt
        speed = min(speed, max_allowed_speed)
        return unit_direction * speed

    def step(self):
        vel=self.velocity_vector()
        self.pos += vel*self.dt
        self.path_x.append(self.pos[0])
        self.path_y.append(self.pos[1])

        return self.pos, self.path_x, self.path_y
