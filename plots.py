import matplotlib.pyplot as plt

class Plot2D:
    def __init__(self, width, height, start, goal):
        self.path_x=[start[0]]
        self.path_y=[start[1]]
        self.measured_path_x=[start[0]]
        self.measured_path_y=[start[1]]
        self.kf_path_x = [start[0]]
        self.kf_path_y = [start[1]]
        self.pred_path_x = [start[0]]
        self.pred_path_y = [start[1]]
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.ax.set_xlim(-width, 10)
        self.ax.set_ylim(-30, 10)
        self.ax.grid(True)
        self.ax.set_title("2D Plane World")

        self.ax.scatter(goal[0], goal[1], c='black')  # goal
        self.pos, = self.ax.plot(start[0], start[1], 'ro', label="True Position")
        self.measured_pos, = self.ax.plot(start[0], start[1], 'bx', label="Measured Position")
        self.kf_pos_plot, = self.ax.plot(start[0], start[1], 'go', label="Kalman Estimate")
        self.pred_pos_plot, = self.ax.plot(start[0], start[1], 'o',color='brown', label="Predicted Estimate")
        self.path_line, = self.ax.plot([], [], 'r-')
        self.measured_path_line, = self.ax.plot([], [], 'b--')
        self.kf_path_line, = self.ax.plot([], [], 'g--')
        self.pred_path_line, = self.ax.plot([], [], '--',color='brown')
        self.ax.legend()

        #speed plot
        self.fig_speed, self.ax_speed = plt.subplots()
        self.ax_speed.set_title("True speed vs Time")
        self.ax_speed.set_xlabel("Time step")
        self.ax_speed.set_ylabel("Speed")
        self.ax_speed.grid(True)

        self.speed_history = []
        self.speed_line, = self.ax_speed.plot([], [], 'g-')

    def update(self, pos, measured_pos, kf_pos, pred_pos, speed):
        self.path_x.append(pos[0])
        self.path_y.append(pos[1])
        self.measured_path_x.append(measured_pos[0])
        self.measured_path_y.append(measured_pos[1])
        self.kf_path_x.append(kf_pos[0])
        self.kf_path_y.append(kf_pos[1])
        self.pred_path_x.append(pred_pos[0])
        self.pred_path_y.append(pred_pos[1])

        self.pos.set_data(pos[0], pos[1])
        self.measured_pos.set_data(measured_pos[0], measured_pos[1])
        self.kf_pos_plot.set_data(kf_pos[0], kf_pos[1])
        self.pred_pos_plot.set_data(pred_pos[0], pred_pos[1])

        self.path_line.set_data(self.path_x, self.path_y)
        self.measured_path_line.set_data(self.measured_path_x, self.measured_path_y)
        self.kf_path_line.set_data(self.kf_path_x, self.kf_path_y)
        self.pred_path_line.set_data(self.pred_path_x, self.pred_path_y)
        
        # Update speedocity plot
        self.speed_history.append(speed)
        self.speed_line.set_data(
            range(len(self.speed_history)),
            self.speed_history
        )

        self.ax_speed.relim()
        self.ax_speed.autoscale_view()

        plt.pause(0.1)

    def show(self):
        plt.show()

class ErrorPlotter:
    def __init__(self):
        self.measured_error_hist = []
        self.pred_error_hist = []
        self.kf_error_hist = []

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Position Estimation Error vs Time")
        self.ax.set_xlabel("Time step")
        self.ax.set_ylabel("Error (Euclidean distance)")
        self.ax.grid(True)

        self.me_line, = self.ax.plot([], [], 'b--', label="Measurement Error (GPS)")
        self.pred_line, = self.ax.plot([], [], '--', color='brown', label="Prediction Error (Model)")
        self.kf_line, = self.ax.plot([], [], 'g-', label="Kalman Error")

        self.ax.legend()

    def update(self, measured_error, pred_error, kf_error):
        self.measured_error_hist.append(measured_error)
        self.pred_error_hist.append(pred_error)
        self.kf_error_hist.append(kf_error)

        t = range(len(self.measured_error_hist))

        self.me_line.set_data(t, self.measured_error_hist)
        self.pred_line.set_data(t, self.pred_error_hist)
        self.kf_line.set_data(t, self.kf_error_hist)

        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.05)

    def show(self):
        plt.show()
