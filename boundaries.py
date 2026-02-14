import numpy as np


class BoundaryConstraints:
    """
    Handles different types of boundary constraints for 2D navigation.
    Supports: hard walls, soft boundaries, and wraparound (toroidal).
    """
    
    def __init__(self,radius,max_speed,dt, x_min, x_max, y_min, y_max):
        """
        Initialize boundary constraints.
        
        Args:
            x_min, x_max: X-axis boundaries
            y_min, y_max: Y-axis boundaries
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.radius=radius
        self.max_speed=max_speed
        self.dt=dt
    
    def is_outside(self, pos):
        """Check if position is within boundaries."""
        if (pos[0] - self.radius < self.x_min or 
            pos[0] + self.radius > self.x_max or
            pos[1] - self.radius < self.y_min or 
            pos[1] + self.radius > self.y_max):
            return True
        return False
    
    def corrected_boundary_violated(self,pos,old_pos):
        movement=pos-old_pos
        corrected_pos = pos.copy()

        if corrected_pos[0] - self.radius < self.x_min:
            corrected_pos[0] = self.x_min + self.radius
        elif corrected_pos[0] + self.radius > self.x_max:
            corrected_pos[0] = self.x_max - self.radius
        
        # Check Y boundaries
        if corrected_pos[1] - self.radius < self.y_min:
            corrected_pos[1] = self.y_min + self.radius
        elif corrected_pos[1] + self.radius > self.y_max:
            corrected_pos[1] = self.y_max - self.radius
        
        return corrected_pos

class ObstacleConstraints:
    """
    Handles rectangular obstacles in the environment.
    """
    
    def __init__(self, max_speed, dt, radius ):
        """
        Initialize obstacle constraints.
        
        Args:
                              'multi_direction' (try down, left, right, up)
        """
        self.obstacles = []
        self.max_speed=max_speed
        self.dt=dt
        self.radius=radius

    def add_obstacle(self, x_min, x_max, y_min, y_max):
        """Add a rectangular obstacle."""
        self.obstacles.append({
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max
        })
    
    def is_colliding(self, pos):
        """Check if position collides with any obstacle."""
        for obs in self.obstacles:
            if (obs['x_min'] - self.radius <= pos[0] <= obs['x_max'] + self.radius and
                obs['y_min'] - self.radius <= pos[1] <= obs['y_max'] + self.radius):
                return True
        return False
    
    def _multi_direction_collision(self, new_pos, old_pos):
        """
        Slide along the obstacle edge when collision detected.
        Decomposes motion into x and y components and tries each separately.
        
        Returns:
            corrected_pos: Position after attempting to slide along obstacles
        """
        # If no collision, return new position as-is
        if not self.is_colliding(new_pos):
            return new_pos.copy()
        
        # Option 1: Try moving only in X direction (slide along Y axis)
        try_x = old_pos.copy()
        try_x[0] = new_pos[0]  # Keep new X, revert Y
        if not self.is_colliding(try_x):
            return try_x
        
        # Option 2: Try moving only in Y direction (slide along X axis)
        try_y = old_pos.copy()
        try_y[1] = new_pos[1]  # Keep new Y, revert X
        if not self.is_colliding(try_y):
            return try_y
        
        # Both sliding attempts failed - stay at old position
        return old_pos.copy()