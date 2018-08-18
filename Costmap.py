import numpy as np
import matplotlib.pyplot as plt

class Costmap:
    def __init__(self, size_y, size_x, resolution = 0.1):
        # x_cordinates = np.random.randint(500, 600, size=500)
        # y_cordinates = np.random.randint(0, 100, size=500)

        self.map = np.zeros((size_x, size_y))
        for x in range(100, 150):
            for y in range(150, 175):
                self.map[x][y] = 80

        for x in range(20, 70):
            for y in range(150, 175):
                self.map[x][y] = 80

        for x in range(150, 175):
            for y in range(100, 125):
                self.map[x][y] = 80

        for x in range(75, 100):
            for y in range(100, 125):
                self.map[x][y] = 80

        for x in range(75, 100):
            for y in range(50, 75):
                self.map[x][y] = 80

        for x in range(0, 230):
                self.map[-x][x] = 100

        self.obstacles = [(125, 162.5), (162.5, 112.5), (45, 162.5), (87.5, 112.5), (87.5, 62.5)]
        self.y = size_y
        self.x = size_x
        self.resolution = resolution

    def display(self):
        plt.imshow(self.map)
        plt.show()

    def get_cell(self, x, y):
        if x >= self.x or y >= self.y or x < -self.x or y < -self.y:
            return None
        return self.map[x][y]

    def set_cell(self, x, y, val):
        if x >= self.x or y >= self.y or x < -self.x or y < -self.y:
            return
        self.map[x][y] = val

