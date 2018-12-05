import numpy as np
import matplotlib.pyplot as plt


class Costmap:
    def __init__(self, size_y, size_x, resolution = 0.1):
        self.map = np.zeros((size_x, size_y))
        for x in range(100, 150):
            for y in range(150, 175):
                self.map[x][y] = 80
        #
        # for x in range(0, 130):
        #     self.map[int(x/3)][x] = 100
        # for x in range(43, 200):
        #     self.map[x][int(120+x/5)] = 100
        for x in range(0, 120):
            for y in range(150, 175):
                self.map[x][y] = 80

        # for x in range(150, 175):
        #     for y in range(100, 125):
        #         self.map[x][y] = 80

        for x in range(75, 100):
            for y in range(0, 130):
                self.map[x][y] = 80

        for x in range(75, 100):
            for y in range(50, 75):
                self.map[x][y] = 80

        # for x in range(0, 230):
        #         self.map[x][x] = 5

        self.obstacles = [(125, 162.5), (162.5, 112.5), (45, 162.5), (87.5, 112.5), (87.5, 62.5)]
        self.y = size_y
        self.x = size_x
        self.resolution = resolution

    def display(self, close=True):
        plt.imshow(self.map)
        plt.show()

    def get_cell(self, x, y):
        if x >= self.x or y >= self.y or x < 0 or y < 0:
            return None
        return self.map[x][y]

    def set_cell(self, x, y, val):
        if x >= self.x or y >= self.y or x < 0 or y < 0:
            return
        self.map[x][y] = val
