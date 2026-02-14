from simulator import Simulator2D
from sensors import PositionSensor
from planner import Planner2D
from plots import Plot2D
from plots import ErrorPlotter
from filters import Kalman2D
from boundaries import BoundaryConstraints, ObstacleConstraints
import numpy as np
import random

WIDTH, HEIGHT = 100, 100
GOAL_TOLERANCE = 1.0

sensor_noise_std = 0.5
max_speed = 2
dt = 1.0
mu = 40.0
sigma = 2.0
motion_noise_std = 0.1
plotting_on = 0  # Set to 1 to see the boundaries in action
radius=0.5
MAX_STEPS = 500   # safety cap

if plotting_on == 1:
    N_RUNS = 1      # Monte Carlo runs
else: 
    N_RUNS = 500

def error_computation(pos, measured_pos, kf_pos, pred_pos):
    measured_error = np.linalg.norm(measured_pos - pos)
    kf_error = np.linalg.norm(kf_pos - pos)
    pred_error = np.linalg.norm(pred_pos - pos)
    return measured_error, kf_error, pred_error

all_meas_errors = []
all_kf_errors = []
all_pred_errors = []
all_collision_counts = []

for run in range(N_RUNS):
    print(f"Monte Carlo run {run+1}/{N_RUNS}")

    if plotting_on == 1:
        start = [0, -10]
        goal = [-98, -68]
    else:
        start = np.array([random.randint(-100, 100), random.randint(1, 100)])
        goal = np.array([random.randint(-100, 100), random.randint(-100, 100)])

    print(f"start {start}")
    print(f"goal {goal}")

    # ============================================================
    # BOUNDARY CONFIGURATION
    # ============================================================
    # Define boundary limits
    boundary_constraints = BoundaryConstraints(
        radius,
        max_speed,
        dt,
        x_min=-100, 
        x_max=10, 
        y_min=-100, 
        y_max=10
    )

    # Optional: Add obstacles
    obstacle_constraints = ObstacleConstraints(max_speed,dt,radius)
    # Uncomment to add obstacles:
    obstacle_constraints.add_obstacle(x_min=-50, x_max=-30, y_min=-50, y_max=-30)
    obstacle_constraints.add_obstacle(x_min=-20, x_max=-10, y_min=-20, y_max=10)

    # To disable boundaries, set boundary_constraints=None
    # To disable obstacles, set obstacle_constraints=None
    # ============================================================

    sim = Simulator2D(
        start,
        goal,
        dt,
        motion_noise_std,
        max_speed,
        boundary_constraints=boundary_constraints,
        obstacle_constraints=obstacle_constraints
    )

    sensor = PositionSensor(sensor_noise_std, start)

    planner = Planner2D(
        goal,
        max_speed,
        dt,
        mu,
        sigma
    )

    if plotting_on == 1:
        # IMPORTANT: Pass boundary_constraints and obstacle_constraints here!
        plotter = Plot2D(
            WIDTH,
            HEIGHT,
            start,
            goal,
            boundary_constraints=boundary_constraints,
            obstacle_constraints=obstacle_constraints
        )

        #error_plotter = ErrorPlotter()

    kf = Kalman2D(
        start,
        motion_noise_std ** 2,
        sensor_noise_std ** 2,
        dt
    )

    vel = planner.velocity_vector(start)

    meas_error_history = []
    kf_error_history = []
    pred_error_history = []

    step_count = 0
    while True:
        measured_pos = sensor.sense_position(sim.pos)
        kf_pos, pred_pos, k = kf.update(measured_pos, vel, dt)
        vel = planner.velocity_vector(kf_pos)
        vel_noisy = sim.add_motion_noise(vel)
        pos, speed = sim.step(vel_noisy)
        
        # Error computation
        measured_error, kf_error, pred_error = error_computation(pos, measured_pos, kf_pos, pred_pos)
        meas_error_history.append(measured_error)
        kf_error_history.append(kf_error)
        pred_error_history.append(pred_error)

        if plotting_on == 1:
            plotter.update(pos, measured_pos, kf_pos, pred_pos, speed, sim.get_collision_count())
            #error_plotter.update(measured_error, pred_error, kf_error)

            k_scalar = np.trace(k) / 4.0
            #plotter.kalman_plot_update(k_scalar)

        step_count += 1
        if step_count > MAX_STEPS:
            print(f"Max steps ({MAX_STEPS}) reached!")
            break

        if sim.distance_to_goal(pred_pos) <= GOAL_TOLERANCE:
            print("Goal reached!")
            break

    # Final update
    kf_pos, pred_pos, k = kf.update(measured_pos, vel, dt)
    vel_noisy = sim.add_motion_noise(vel)
    
    if plotting_on == 1:
        plotter.update(pos, measured_pos, kf_pos, pred_pos, speed, sim.get_collision_count())
        
        k_scalar = np.trace(k) / 4.0
        #plotter.kalman_plot_update(k_scalar)
        plotter.show()

    all_meas_errors.append(meas_error_history)
    all_kf_errors.append(kf_error_history)
    all_pred_errors.append(pred_error_history)
    all_collision_counts.append(sim.get_collision_count())

# Statistics
print(f"\n{'='*50}")
print(f"Total Collisions across all runs: {sum(all_collision_counts)}")
print(f"Average Collisions per run: {np.mean(all_collision_counts):.2f}")
print(f"{'='*50}\n")

# Convert to numpy (ragged runs → pad or truncate)
min_len = min(len(e) for e in all_kf_errors)

all_meas_errors = np.array([e[:min_len] for e in all_meas_errors])
all_kf_errors = np.array([e[:min_len] for e in all_kf_errors])
all_pred_errors = np.array([e[:min_len] for e in all_pred_errors])

# RMSE over Monte Carlo runs (square, mean, then sqrt)
rmse_meas = np.sqrt(np.mean(all_meas_errors ** 2, axis=0))
rmse_kf = np.sqrt(np.mean(all_kf_errors ** 2, axis=0))
rmse_pred = np.sqrt(np.mean(all_pred_errors ** 2, axis=0))

# Overall RMSE (across all timesteps and runs)
overall_rmse_meas = np.sqrt(np.mean(all_meas_errors ** 2))
overall_rmse_kf = np.sqrt(np.mean(all_kf_errors ** 2))
overall_rmse_pred = np.sqrt(np.mean(all_pred_errors ** 2))

print("Overall RMSE GPS error:", overall_rmse_meas)
print("Overall RMSE KF error:", overall_rmse_kf)
print("Overall RMSE Prediction error:", overall_rmse_pred)

# If you want time-averaged RMSE (RMSE at each timestep, then average)
time_avg_rmse_meas = np.mean(rmse_meas)
time_avg_rmse_kf = np.mean(rmse_kf)
time_avg_rmse_pred = np.mean(rmse_pred)

print("\nTime-averaged RMSE GPS error:", time_avg_rmse_meas)
print("Time-averaged RMSE KF error:", time_avg_rmse_kf)
print("Time-averaged RMSE Prediction error:", time_avg_rmse_pred)