import numpy as np
from boundaries import BoundaryConstraints, ObstacleConstraints

class Simulator2D:
    def __init__(self, start, goal, dt, motion_noise_std, max_speed, 
                 boundary_constraints, obstacle_constraints=None):
        self.pos = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)

        self.dt = dt
        self.motion_noise_std = motion_noise_std
        self.max_speed = max_speed

        # Velocity tracking
        self.vel = np.zeros(2)

        self.boundarycon=boundary_constraints
        self.obscon=obstacle_constraints

        # Logging
        self.path_x = [self.pos[0]]
        self.path_y = [self.pos[1]]
        
        # Collision tracking
        self.collision_count = 0

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
        """
        Execute one simulation step.
        
        Returns:
            pos: New position
            speed: Current speed
        """
        old_pos = self.pos.copy()
        
        # Compute desired new position
        new_pos = self.pos + vel_noisy * self.dt
        
        if self.obscon is not None and self.obscon.is_colliding(new_pos):
            new_pos = self.obscon._multi_direction_collision(new_pos, old_pos)
            self.collision_count += 1
        
        # Apply boundary constraints (if they exist)
        if self.boundarycon is not None and self.boundarycon.is_outside(new_pos):
            new_pos = self.boundarycon.corrected_boundary_violated(new_pos, old_pos)
            self.collision_count += 1

        # Logging
        self.path_x.append(new_pos[0])
        self.path_y.append(new_pos[1])
        self.pos=new_pos.copy()

        speed = np.linalg.norm(vel_noisy)
        return self.pos, speed
    
    def get_collision_count(self):
        """Return number of collisions."""
        return self.collision_count