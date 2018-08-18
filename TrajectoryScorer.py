import numpy as np
from Costmap import Costmap
import math
import copy
import os
from GlobalPath import GlobalPath
from Pose import Pose


class TrajectoryScorer:

    def __init__(self, costmap):
        self.costmap = costmap

        # Generate global path
        poses = []
        for x in range(costmap.x):
            poses.append(Pose(-x, x))
        self.global_path = GlobalPath(poses)

    def trajectory_cost(self, trajectory):
        cost = 0

        # Trajectory cost scoring function
        costmap_cost = self.costmap_cost(trajectory)
        # obstacle_dist_cost = self.obstacle_dist_cost(trajectory)
        global_path_cost = self.global_path_cost(trajectory)
        # Computing combined
        print(costmap_cost*100, global_path_cost)
        cost = costmap_cost*100 + global_path_cost

        trajectory.cost = cost
        self.obstacle_dist_cost(trajectory)

        return cost

    def costmap_cost(self, trajectory):
        cost = 0
        prev_pos = (-1, -1)
        for pose in trajectory.poses:
            position = (math.floor(pose.x / self.costmap.resolution), math.floor(pose.y / self.costmap.resolution))
            if prev_pos != position:
                cost += self.costmap.map[position[0]][position[1]]
            prev_pos = position
        return cost

    def obstacle_dist_cost(self, trajectory):
        mid = len(trajectory.poses)
        mid_pose = trajectory.poses[int(mid / 2)]
        dist = []
        for obstacle in self.costmap.obstacles:
            dist.append(math.sqrt(math.fabs(((mid_pose.x - obstacle[0])**2) - ((mid_pose.y - obstacle[1])**2))))

        closes_dist = 0

        for i in range(0, 3):
            min_dist = min(dist)
            closes_dist += (-min_dist)
            dist.remove(min_dist)

        return closes_dist

    def global_path_cost(self, trajectory):
        cost = []
        for pose in trajectory.poses:
            min_dist = 1e5
            for global_pose in self.global_path.poses:
                dist = TrajectoryScorer.distance(pose, global_pose)
                if dist < min_dist:
                    min_dist = dist
            cost.append(min_dist)

        return sum(cost)

    @staticmethod
    def distance(pose1, pose2):
        return math.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)

    def best_trajectory(self, trajectories, num_best_traj):
        trajectory_list = []
        best_trajectories = []
        for trajectory in trajectories:
            cost = trajectory.score(self.costmap)
            trajectory_list.append((cost, trajectory))

        if int(os.environ.get("DISPLAY_PATH_SEARCH")):
            vis_costmap = copy.deepcopy(self.costmap)
            for trajectory in trajectories:
                trajectory.visualize(vis_costmap, 10)
            vis_costmap.display()

        for i in range(0, num_best_traj):
            min_cost_trajectory = min(trajectory_list, key=lambda t: t[0])
            best_trajectories.append(min_cost_trajectory[1])
            trajectory_list.remove(min_cost_trajectory)

        return best_trajectories
