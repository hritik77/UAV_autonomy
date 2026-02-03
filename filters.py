import numpy as np

class Kalman:
    def __init__(self, start, process_var, meas_var):
        # Estimated position
        self.kf_pos = np.array(start, dtype=float)

        # Uncertainty (covariance, simplified scalar)
        self.P = 1.0

        # Noise models
        self.Q = process_var      # process noise (motion uncertainty)
        self.R = meas_var         # measurement noise (GPS uncertainty)

        #Kalman Gain
        self.P_pred = self.P + self.Q
        self.K = self.P_pred / (self.P_pred + self.R)

        print(f"Kalman")

    def update(self, measured_pos, velocity, dt):
        """
        measured_pos : GPS position (noisy)
        velocity     : commanded / estimated velocity
        dt           : timestep
        """

        # ---------- PREDICTION STEP ----------
        pred_pos = self.kf_pos + velocity * dt

        # ---------- UPDATE STEP ----------
        self.kf_pos = pred_pos + self.K * (measured_pos - pred_pos)
        self.P = (1 - self.K) * self.P_pred

        return self.kf_pos,pred_pos
