from simulator import Simulator2D
from plots import Plot2D

WIDTH, HEIGHT = 100, 100
GOAL_TOLERANCE=0.0

sim = Simulator2D(
    start=[0.0, 0.0],
    goal=[-96, -28],
    max_speed=10,
    dt=1.0,
    mu=40.0,
    sigma=20.0
)

plotter = Plot2D(
    width=WIDTH,
    height=HEIGHT,
    start=sim.pos,
    goal=sim.goal
)

while True:
    pos, path_x, path_y = sim.step()
    plotter.update(pos, path_x, path_y)
    if sim.distance_to_goal() <= GOAL_TOLERANCE:
        print("Goal reached!")
        break

plotter.show()