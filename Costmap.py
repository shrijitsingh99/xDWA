import numpy as np
import matplotlib.pyplot as plt

class Costmap:
    def __init__(self, size_y, size_x, resolution = 0.1):
        # x_cordinates = np.random.randint(500, 600, size=500)
        # y_cordinates = np.random.randint(0, 100, size=500)

        self.map = np.zeros((size_y, size_x))
        for x in range(550, 650):
            for y in range(0, 100):
                self.map[x][y] = 0
        self.y = size_y
        self.x = size_x
        self.resolution = resolution

    def visualize(self):
        plt.imshow(self.map)
        plt.show()