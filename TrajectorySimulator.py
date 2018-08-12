import numpy as np
import math
from Pose import Pose
from Trajectory import Trajectory
from Velocity import Velocity

class TrajectorySimulator:
    def __init__(self, time, timestep):
        self.time = time
        self.timestep = timestep

    def simulate_trajectory(self, robot, vx, vtheta):
        poses = []
        current_pose = [robot.pose.x, robot.pose.y, robot.pose.theta]
        for i in range(0, int(self.time / self.timestep)):
            current_pose[0] += (vx * math.cos(current_pose[2])) * self.timestep
            current_pose[1] += (vx * math.sin(current_pose[2])) * self.timestep
            current_pose[2] += vtheta * self.timestep
            new_pose = Pose(current_pose[0], current_pose[1], current_pose[2])
            poses.append(new_pose)
        return Trajectory(poses=poses, velocity=Velocity(vx, vtheta), cost=0)