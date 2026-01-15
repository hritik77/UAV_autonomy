from simulator import Simulator2D
from plots import Plot2D

WIDTH, HEIGHT = 100, 100

sim = Simulator2D(
    start=[0.0, 0.0],
    goal=[81.5, 67.3],
    velocity=[2.0, 3.0]
)

plotter = Plot2D(
    width=WIDTH,
    height=HEIGHT,
    start=sim.pos,
    goal=sim.goal
)

for _ in range(30):
    pos, path_x, path_y = sim.step()
    plotter.update(pos, path_x, path_y)

plotter.show()
