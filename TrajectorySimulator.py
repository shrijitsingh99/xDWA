import math
from Pose import Pose
from Velocity import Velocity
import Trajectory as traj


class TrajectorySimulator:

    def __init__(self, time, timestep, costmap):
        self.time = time
        self.timestep = timestep
        self.costmap = costmap

    def simulate_trajectory(self, robot, vx, vtheta):
        poses = []
        # prev_pose = Pose(0, 0, 0)
        current_pose = [robot.pose.x, robot.pose.y, robot.pose.theta]
        for i in range(0, int(self.time / self.timestep)):
            current_pose[0] += (vx * math.cos(current_pose[2])) * self.timestep
            current_pose[1] += (vx * math.sin(current_pose[2])) * self.timestep
            current_pose[2] += vtheta * self.timestep
            new_pose = Pose(current_pose[0], current_pose[1], current_pose[2])
            poses.append(new_pose)

            # current_pose[2] += vtheta * self.timestep
            # new_pose = Pose(math.floor(current_pose[0]) / self.costmap.resolution, math.floor(current_pose[1]) / self.costmap.resolution, current_pose[2])
            # if prev_pose.x == new_pose.x and prev_pose.y == new_pose.y:
            #     continue
            # poses.append(new_pose)
            # prev_pose.x = new_pose.x
            # prev_pose.y = new_pose.y
        return traj.Trajectory(poses=poses, velocity=Velocity(vx, vtheta), cost=0)
