import numpy as np

class Planner2D:
    def __init__(self, goal, max_speed, dt, mu, sigma, motion_noise_std):
        self.goal = np.array(goal, dtype=float)
        self.max_speed = max_speed
        self.dt = dt
        self.mu = mu
        self.sigma = sigma
        self.motion_noise_std = motion_noise_std

    def gaussian_speed(self, distance):
        return self.max_speed * np.exp(
            -((distance - self.mu) ** 2) / (2 * self.sigma ** 2)
        )

    def constant_speed():
        return 2
    
    def add_motion_noise(self, vel):
        noise = np.random.normal(
            loc=0.0,
            scale=self.motion_noise_std,
            size=2
        )
        return vel + noise

    def velocity_vector(self, measured_pos):
        direction = self.goal - measured_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return np.zeros(2)

        unit_direction = direction / distance
        #speed = self.gaussian_speed(distance)
        speed = Planner2D.constant_speed()

        max_allowed_speed = distance / self.dt
        speed = min(speed, max_allowed_speed)

        vel = unit_direction * speed
        return self.add_motion_noise(vel)