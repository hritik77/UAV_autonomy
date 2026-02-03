from simulator import Simulator2D
from sensors import PositionSensor
from planner import Planner2D
from plots import Plot2D
from plots import ErrorPlotter
from filters import Kalman
import numpy as np

WIDTH, HEIGHT = 100, 100
GOAL_TOLERANCE=1.0

start=[0.0, 0.0]
goal=[-96, -28]
sensor_noise_std=0.5
max_speed=2
dt=1.0
mu=40.0
sigma=2.0
motion_noise_std=0.05

sim = Simulator2D(
    start,
    goal,
    dt,
    motion_noise_std,
    max_speed
)

sensor = PositionSensor(sensor_noise_std, start)

planner = Planner2D(
    goal,
    max_speed,
    dt,
    mu,
    sigma
)

plotter = Plot2D(
    WIDTH,
    HEIGHT,
    start,
    goal
)

kf = Kalman(start,
            motion_noise_std ** 2,
            sensor_noise_std ** 2
)

error_plotter = ErrorPlotter()

vel = planner.velocity_vector(start)

def error_computation(pos,measured_pos,kf_pos,pred_pos) :
    measured_error = np.linalg.norm(measured_pos - pos)
    kf_error = np.linalg.norm(kf_pos - pos)
    pred_error = np.linalg.norm(pred_pos - pos)
    return measured_error,kf_error,pred_error

meas_error_history = []
kf_error_history = []
pred_error_history = []

while True:
    measured_pos = sensor.sense_position(sim.pos)
    kf_pos,pred_pos = kf.update(measured_pos,vel,dt)
    vel = planner.velocity_vector(kf_pos)
    vel_noisy = sim.add_motion_noise(vel)
    pos, speed = sim.step(vel_noisy)
    plotter.update(pos, measured_pos, kf_pos, pred_pos, speed)
    
    #error computation
    measured_error,kf_error,pred_error = error_computation(pos,measured_pos,kf_pos,pred_pos)
    meas_error_history.append(measured_error)
    kf_error_history.append(kf_error)
    pred_error_history.append(pred_error)
    error_plotter.update(measured_error,pred_error,kf_error)


    #print(f"Vel: {vel}  vel_noisy: {vel_noisy} \n")    

    if sim.distance_to_goal(pred_pos) <= GOAL_TOLERANCE:
        print("Goal reached!")
        break

kf_pos,pred_pos = kf.update(measured_pos,vel,dt)
vel_noisy = sim.add_motion_noise(vel)
plotter.update(pos, measured_pos, kf_pos, pred_pos, speed)
plotter.show()