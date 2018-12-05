import numpy as np
import math
from TrajectoryScorer import TrajectoryScorer
from SampleGenerator import SampleGenerator
from TrajectorySimulator import TrajectorySimulator


class Trajectory:

    def __init__(self, poses, velocity, cost=0):
        self.poses = np.array(poses)
        self.cost = cost
        self.velocity = velocity

    def score(self, costmap):
        scorer = TrajectoryScorer(costmap)
        return scorer.trajectory_cost(self)

    @classmethod
    def generate_trajectories(cls, robot, costmap, depth):
        if depth == 0:
            generator = SampleGenerator(robot, timestep=0.1, resolution_x=0.1, resolution_theta=0.1)
        else:
            generator = SampleGenerator(robot, timestep=0.5, resolution_x=0.4, resolution_theta=0.2)
        vel_samples = generator.generate()
        simulator = TrajectorySimulator(time=4, timestep=0.1, costmap=costmap)
        trajectories = []
        for vel in vel_samples:
            trajectories.append(simulator.simulate_trajectory(robot, vel.x, vel.theta))
            
        return trajectories

    def visualize(self, costmap, i, display=False):
            for pose in self.poses:
                position = (math.floor(pose.x / costmap.resolution), math.floor(pose.y / costmap.resolution))
                costmap.set_cell(*position, (i+1)*10)
                if display:
                    costmap.visualize()
