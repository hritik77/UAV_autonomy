import numpy as np

class Planner2D:
    def __init__(self, goal, max_speed, dt, mu, sigma):
        self.goal = np.array(goal, dtype=float)
        self.max_speed = max_speed
        self.dt = dt
        self.mu = mu
        self.sigma = sigma

    def gaussian_speed(self, distance):
        return self.max_speed * np.exp(
            -((distance - self.mu) ** 2) / (2 * self.sigma ** 2)
        )+self.max_speed/2

    def constant_speed(self):
        return min(self.max_speed,2)

    def velocity_vector(self, kf_pos):
        direction = self.goal - kf_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return np.zeros(2)

        unit_direction = direction / distance
        #speed = self.gaussian_speed(distance)
        #print(f"Gauss: {speed}")
        speed = Planner2D.constant_speed(self)
        speed = min(speed,self.max_speed)
        
        max_allowed_speed = distance / self.dt
        speed = min(speed, max_allowed_speed)

        vel = unit_direction * speed
        return vel