from simulator import Simulator2D
from sensors import PositionSensor
from planner import Planner2D
from plots import Plot2D

WIDTH, HEIGHT = 100, 100
GOAL_TOLERANCE=1.0

start=[0.0, 0.0]
goal=[-96, -28]
sensor_noise_std=0.5
max_speed=10
dt=1.0
mu=40.0
sigma=20.0
motion_noise_std=0.2

sim = Simulator2D(
    start,
    goal,
    dt
)

sensor = PositionSensor(sensor_noise_std, start)

planner = Planner2D(
    goal,
    max_speed,
    dt,
    mu,
    sigma,
    motion_noise_std
)

plotter = Plot2D(
    width=WIDTH,
    height=HEIGHT,
    start=sim.pos,
    goal=sim.goal
)

while True:
    measured_pos = sensor.sense_position(sim.pos)
    vel_noisy = planner.velocity_vector(measured_pos)
    pos, speed = sim.step(vel_noisy)

    plotter.update(pos, measured_pos, speed)

    if sim.distance_to_goal() <= GOAL_TOLERANCE:
        print("Goal reached!")
        break

plotter.show()