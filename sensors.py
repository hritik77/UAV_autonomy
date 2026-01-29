import numpy as np

class PositionSensor:
    def __init__(self, sensor_noise_std, start):
        self.sensor_noise_std = sensor_noise_std
        self.measured_pos = start

        self.measured_path_x = [start[0]]
        self.measured_path_y = [start[1]]

    def sense_position(self, true_pos):
        noise = np.random.normal(
            loc=0.0,
            scale=self.sensor_noise_std,
            size=2
        )
        self.measured_pos = true_pos + noise

        self.measured_path_x.append(self.measured_pos[0])
        self.measured_path_y.append(self.measured_pos[1])

        return self.measured_pos