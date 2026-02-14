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

        self.P_pred = self.P + self.Q

    def update(self, measured_pos, velocity, dt):
        """
        measured_pos : GPS position (noisy)
        velocity     : commanded / estimated velocity
        dt           : timestep
        """

        # ---------- PREDICTION STEP ----------
        pred_pos = self.kf_pos + velocity * dt
        P_pred = self.P + self.Q

        K = P_pred / (P_pred + self.R)

        # ---------- UPDATE STEP ----------
        self.kf_pos = pred_pos + K * (measured_pos - pred_pos)
        self.P = (1 - K) * self.P_pred

        return self.kf_pos,pred_pos,K
    
class Kalman2D:
    def __init__(self, start, process_var, meas_var, dt):
        # State: [x, y, vx, vy]
        self.x = np.array([start[0], start[1], 0.0, 0.0])
        
        # Covariance matrix (4x4)
        self.P = np.eye(4) * 1.0
        
        # State transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Measurement matrix (we measure position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        q = process_var
        self.Q = np.array([
            [dt**4/4*q, 0, dt**3/2*q, 0],
            [0, dt**4/4*q, 0, dt**3/2*q],
            [dt**3/2*q, 0, dt**2*q, 0],
            [0, dt**3/2*q, 0, dt**2*q]
        ])
        
        # Measurement noise covariance
        self.R = np.eye(2) * meas_var
        
    def update(self, measured_pos, control_velocity, dt):
        # Optional: add control input (if you trust motor commands)
        # For now, we let the filter estimate velocity itself
        
        # ========== PREDICTION ==========
        # Predict state
        x_pred = self.F @ self.x
        
        # Predict covariance
        P_pred = self.F @ self.P @ self.F.T + self.Q
        
        # ========== UPDATE ==========
        # Measurement residual
        z = measured_pos  # [x_meas, y_meas]
        y = z - (self.H @ x_pred)  # innovation
        
        # Innovation covariance
        S = self.H @ P_pred @ self.H.T + self.R
        
        # Kalman gain (4x2 matrix)
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = x_pred + K @ y
        
        # Update covariance
        I = np.eye(4)
        self.P = (I - K @ self.H) @ P_pred
        
        # Return position estimate
        return self.x[:2], x_pred[:2], K