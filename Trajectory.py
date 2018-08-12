import numpy as np
import math
from Costmap import Costmap
from Velocity import Velocity
from TrajectoryScorer import TrajectoryScorer

class Trajectory:
    def __init__(self, poses, velocity, cost=0):
        self.poses = np.array(poses)
        self.cost = cost
        self.velocity = velocity

    def score(self, costmap):
        scorer = TrajectoryScorer(costmap)
        return scorer.trajectory_cost(self)

    def visualize(self, costmap, i):
            for pose in self.poses:
                position = (math.floor(pose.x / costmap.resolution), math.floor(pose.y / costmap.resolution))
                costmap.map[position[0]][position[1]] = (i + 1)*100
            # costmap.visualize()



