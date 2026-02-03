import numpy as np

class Simulator2D:
    def __init__(self, start, goal, dt, motion_noise_std, max_speed):
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)

        self.dt = dt
        self.motion_noise_std = motion_noise_std
        self.max_speed = max_speed

        # Logging
        self.path_x = [self.pos[0]]
        self.path_y = [self.pos[1]]

    def distance_to_goal(self, pred_pos):
        return np.linalg.norm(self.goal - pred_pos)

    def add_motion_noise(self, vel):
        noise = np.random.normal(
            loc=0.0,
            scale=self.motion_noise_std,
            size=2
        )

        vel_noisy = vel + noise
        # Clamp speed
        speed = np.linalg.norm(vel_noisy)
        if speed > self.max_speed:
            vel_noisy = (vel_noisy / speed) * self.max_speed

        return vel_noisy

    def step(self, vel_noisy):
        self.pos += vel_noisy * self.dt

        self.path_x.append(self.pos[0])
        self.path_y.append(self.pos[1])

        speed = np.linalg.norm(vel_noisy)
        return self.pos, speed
