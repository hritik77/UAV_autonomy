import matplotlib.pyplot as plt

class Plot2D:
    def __init__(self, width, height, start, goal):
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.ax.set_xlim(-width, width)
        self.ax.set_ylim(-height, height)
        self.ax.grid(True)
        self.ax.set_title("2D Plane World")

        self.ax.scatter(goal[0], goal[1], c='black')  # goal
        self.point, = self.ax.plot(start[0], start[1], 'ro')
        self.path_line, = self.ax.plot([], [], 'b-')

    def update(self, pos, path_x, path_y):
        self.point.set_data(pos[0], pos[1])
        self.path_line.set_data(path_x, path_y)
        plt.pause(0.05)

    def show(self):
        plt.show()