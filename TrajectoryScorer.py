import numpy as np
from Costmap import Costmap
import math


class TrajectoryScorer:
    def __init__(self, costmap):
        self.costmap = costmap

    def trajectory_cost(self, trajectory):
        cost = 0
        prev_pos = (0, 0)
        for pose in trajectory.poses:
            position = (math.floor(pose.x / self.costmap.resolution), math.floor(pose.y / self.costmap.resolution))
            if prev_pos != position:
                cost += self.costmap.map[position[0]][position[1]]
            prev_pos = position
        trajectory.cost = cost
        return cost

