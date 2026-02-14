import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Plot2D:
    """Enhanced Plot2D that can visualize boundaries and obstacles."""
    
    def __init__(self, width, height, start, goal, boundary_constraints=None, obstacle_constraints=None):
        # ---- Path histories ----
        self.path_x = [start[0]]
        self.path_y = [start[1]]

        self.measured_path_x = [start[0]]
        self.measured_path_y = [start[1]]

        self.kf_path_x = [start[0]]
        self.kf_path_y = [start[1]]

        self.pred_path_x = [start[0]]
        self.pred_path_y = [start[1]]

        # ---- Main 2D plot ----
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        
        # Set limits based on boundaries if provided
        if boundary_constraints:
            self.ax.set_xlim(boundary_constraints.x_min - 5, boundary_constraints.x_max + 5)
            self.ax.set_ylim(boundary_constraints.y_min - 5, boundary_constraints.y_max + 5)
        else:
            self.ax.set_xlim(-width, width)
            self.ax.set_ylim(-height, height)
        
        self.ax.grid(True)
        self.ax.set_title("2D Plane World")

        # Draw boundaries
        if boundary_constraints:
            self._draw_boundaries(boundary_constraints)
        
        # Draw obstacles
        if obstacle_constraints:
            self._draw_obstacles(obstacle_constraints)

        # Goal
        self.ax.scatter(goal[0], goal[1], c='black', s=100, marker='*', 
                       label='Goal', zorder=10)

        # Current positions (IMPORTANT: plotted as Line2D → needs sequences)
        self.pos, = self.ax.plot([start[0]], [start[1]], 'ro', 
                                label="True Position", markersize=8)
        self.measured_pos, = self.ax.plot([start[0]], [start[1]], 'bx', 
                                          label="Measured Position", markersize=8)
        self.kf_pos_plot, = self.ax.plot([start[0]], [start[1]], 'go', 
                                         label="Kalman Estimate", markersize=8)
        self.pred_pos_plot, = self.ax.plot([start[0]], [start[1]], 'o', 
                                           color='brown', label="Predicted Estimate", 
                                           markersize=8)

        # Paths
        self.path_line, = self.ax.plot([], [], 'r-', linewidth=2, alpha=0.7)
        self.measured_path_line, = self.ax.plot([], [], 'b--', alpha=0.5)
        self.kf_path_line, = self.ax.plot([], [], 'g--', alpha=0.7)
        self.pred_path_line, = self.ax.plot([], [], '--', color='brown', alpha=0.5)

        self.ax.legend(loc='lower right')

        # ---- Speed vs Time plot ----
        #self.fig_speed, self.ax_speed = plt.subplots()
        #self.ax_speed.set_title("True Speed vs Time")
        #self.ax_speed.set_xlabel("Time step")
        #self.ax_speed.set_ylabel("Speed")
        #self.ax_speed.grid(True)

        #self.speed_history = []
        #self.speed_line, = self.ax_speed.plot([], [], 'g-')

        # ---- Kalman Gain vs Time plot ----
        #self.fig_K, self.ax_K = plt.subplots()
        #self.ax_K.set_title("Kalman Gain vs Time")
        #self.ax_K.set_xlabel("Time step")
        #self.ax_K.set_ylabel("Kalman Gain")
        #self.ax_K.grid(True)

        #self.K_history = []
        #self.K_line, = self.ax_K.plot([], [], 'm-')
        
        # ---- Collision tracking text ----
        self.collision_text = None
        if boundary_constraints or obstacle_constraints:
            self.collision_text = self.ax.text(
                0.02, 0.98,
                "Collisions: 0",
                transform=self.ax.transAxes,
                fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                zorder=20
            )

    def _draw_boundaries(self, boundary_constraints):
        """Draw boundary walls on the plot with enhanced visualization."""
        x_min = boundary_constraints.x_min
        x_max = boundary_constraints.x_max
        y_min = boundary_constraints.y_min
        y_max = boundary_constraints.y_max
        
        boundary_type = 'soft'
        
        if boundary_type == 'hard':
            edge_color = 'red'
            fill_color = 'red'
            linestyle = '-'
            linewidth = 4
            label = 'Hard Boundary (Bounce)'
        elif boundary_type == 'soft':
            edge_color = 'darkorange'
            fill_color = 'orange'
            linestyle = '-'
            linewidth = 3
            label = 'Soft Boundary (Clamp)'
        elif boundary_type == 'wraparound':
            edge_color = 'blue'
            fill_color = 'blue'
            linestyle = '--'
            linewidth = 3
            label = 'Wraparound Boundary'
        else:
            edge_color = 'gray'
            fill_color = 'gray'
            linestyle = '-'
            linewidth = 2
            label = 'Boundary'
        
        # Draw shaded region outside boundaries to make them more visible
        plot_xlim = self.ax.get_xlim()
        plot_ylim = self.ax.get_ylim()
        
        # Left shaded region (outside boundary)
        if x_min > plot_xlim[0]:
            left_rect = patches.Rectangle(
                (plot_xlim[0], plot_ylim[0]),
                x_min - plot_xlim[0],
                plot_ylim[1] - plot_ylim[0],
                facecolor=fill_color,
                alpha=0.1,
                edgecolor='none',
                zorder=0
            )
            self.ax.add_patch(left_rect)
        
        # Right shaded region
        if x_max < plot_xlim[1]:
            right_rect = patches.Rectangle(
                (x_max, plot_ylim[0]),
                plot_xlim[1] - x_max,
                plot_ylim[1] - plot_ylim[0],
                facecolor=fill_color,
                alpha=0.1,
                edgecolor='none',
                zorder=0
            )
            self.ax.add_patch(right_rect)
        
        # Bottom shaded region
        if y_min > plot_ylim[0]:
            bottom_rect = patches.Rectangle(
                (plot_xlim[0], plot_ylim[0]),
                plot_xlim[1] - plot_xlim[0],
                y_min - plot_ylim[0],
                facecolor=fill_color,
                alpha=0.1,
                edgecolor='none',
                zorder=0
            )
            self.ax.add_patch(bottom_rect)
        
        # Top shaded region
        if y_max < plot_ylim[1]:
            top_rect = patches.Rectangle(
                (plot_xlim[0], y_max),
                plot_xlim[1] - plot_xlim[0],
                plot_ylim[1] - y_max,
                facecolor=fill_color,
                alpha=0.1,
                edgecolor='none',
                zorder=0
            )
            self.ax.add_patch(top_rect)
        
        # Draw main boundary rectangle with thick border
        boundary_rect = patches.Rectangle(
            (x_min, y_min), 
            x_max - x_min, 
            y_max - y_min,
            linewidth=linewidth,
            edgecolor=edge_color,
            facecolor='none',
            linestyle=linestyle,
            label=label,
            zorder=5
        )
        self.ax.add_patch(boundary_rect)
        
        # Add text annotation at top-left corner showing boundary info
        info_text = f"Boundary: {boundary_type.upper()}\n[{x_min}, {x_max}] × [{y_min}, {y_max}]"
        self.ax.text(
            x_min + 2, y_max - 2,
            info_text,
            fontsize=9,
            color=edge_color,
            fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8, edgecolor=edge_color),
            verticalalignment='top',
            zorder=10
        )

    def _draw_obstacles(self, obstacle_constraints):
        """Draw obstacles on the plot with enhanced visualization."""
        for i, obs in enumerate(obstacle_constraints.obstacles):
            # Draw shadow for depth effect
            shadow = patches.Rectangle(
                (obs['x_min'] + 0.5, obs['y_min'] - 0.5),
                obs['x_max'] - obs['x_min'],
                obs['y_max'] - obs['y_min'],
                linewidth=0,
                edgecolor='none',
                facecolor='black',
                alpha=0.2,
                zorder=1
            )
            self.ax.add_patch(shadow)
            
            # Draw main obstacle
            obstacle = patches.Rectangle(
                (obs['x_min'], obs['y_min']),
                obs['x_max'] - obs['x_min'],
                obs['y_max'] - obs['y_min'],
                linewidth=2.5,
                edgecolor='darkred',
                facecolor='lightcoral',
                alpha=0.7,
                label='Obstacle' if i == 0 else '',
                zorder=2
            )
            self.ax.add_patch(obstacle)
            
            # Add diagonal lines pattern for better visibility
            x_min, x_max = obs['x_min'], obs['x_max']
            y_min, y_max = obs['y_min'], obs['y_max']
            
            # Draw diagonal pattern
            num_lines = 5
            for j in range(num_lines):
                t = j / (num_lines - 1) if num_lines > 1 else 0.5
                self.ax.plot([x_min, x_max], 
                           [y_min + t * (y_max - y_min), y_min + t * (y_max - y_min)],
                           'r-', alpha=0.3, linewidth=0.5, zorder=2)
                self.ax.plot([x_min + t * (x_max - x_min), x_min + t * (x_max - x_min)],
                           [y_min, y_max],
                           'r-', alpha=0.3, linewidth=0.5, zorder=2)

    def kalman_plot_update(self, k):
        self.K_history.append(k)

        t = range(len(self.K_history))
        self.K_line.set_data(t, self.K_history)

        self.ax_K.relim()
        self.ax_K.autoscale_view()

        plt.pause(0.05)

    def update(self, pos, measured_pos, kf_pos, pred_pos, speed, collision_count=None):
        # ---- Store paths ----
        self.path_x.append(pos[0])
        self.path_y.append(pos[1])

        self.measured_path_x.append(measured_pos[0])
        self.measured_path_y.append(measured_pos[1])

        self.kf_path_x.append(kf_pos[0])
        self.kf_path_y.append(kf_pos[1])

        self.pred_path_x.append(pred_pos[0])
        self.pred_path_y.append(pred_pos[1])

        # ---- Update current points (MUST be sequences) ----
        self.pos.set_data([pos[0]], [pos[1]])
        self.measured_pos.set_data([measured_pos[0]], [measured_pos[1]])
        self.kf_pos_plot.set_data([kf_pos[0]], [kf_pos[1]])
        self.pred_pos_plot.set_data([pred_pos[0]], [pred_pos[1]])

        # ---- Update paths ----
        self.path_line.set_data(self.path_x, self.path_y)
        self.measured_path_line.set_data(self.measured_path_x, self.measured_path_y)
        self.kf_path_line.set_data(self.kf_path_x, self.kf_path_y)
        self.pred_path_line.set_data(self.pred_path_x, self.pred_path_y)

        # ---- Speed plot ----
        #self.speed_history.append(speed)
        #self.speed_line.set_data(
        #    range(len(self.speed_history)),
        #    self.speed_history
        #)

        #self.ax_speed.relim()
        #self.ax_speed.autoscale_view()
        
        # ---- Update collision counter ----
        if self.collision_text is not None and collision_count is not None:
            self.collision_text.set_text(f"Collisions: {collision_count}")

        plt.pause(0.05)

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