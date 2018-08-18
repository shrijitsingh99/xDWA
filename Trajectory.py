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
    def generate_trajectories(cls, robot):
        generator = SampleGenerator(robot, timestep=0.1, resolution_x=0.05, resolution_theta=0.05)
        vel_samples = generator.generate()
        simulator = TrajectorySimulator(time=3, timestep=0.1)
        trajectories = []
        for vel in vel_samples:
            trajectories.append(simulator.simulate_trajectory(robot, vel.x, vel.theta))
            
        return trajectories

    def visualize(self, costmap, i, display=False):
            for pose in self.poses:
                position = (math.floor(pose.x / costmap.resolution), math.floor(pose.y / costmap.resolution))
                costmap.map[position[0]][position[1]] = (i + 1)*100
                if display:
                    costmap.visualize()



