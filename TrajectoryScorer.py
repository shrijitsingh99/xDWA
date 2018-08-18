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
        poses = []
        for x in range(costmap.x):
            poses.append(Pose(x, x))
        self.global_path = GlobalPath(poses)

    def trajectory_cost(self, trajectory):
        costmap_cost = self.costmap_cost(trajectory)
        # obstacle_dist_cost = self.obstacle_dist_cost(trajectory)
        global_path_cost = self.global_path_cost(trajectory)
        cost = costmap_cost + global_path_cost
        trajectory.cost = cost
        self.obstacle_dist_cost(trajectory)

        return cost

    def costmap_cost(self, trajectory):
        cost = 0
        prev_pos = (115, 115)
        # TODO (squadrick): prev_pos should be Robot's starting x and y
        for pose in trajectory.poses:
            pose_cell = 1
            position = (math.floor(pose.x / self.costmap.resolution), math.floor(pose.y / self.costmap.resolution))
            if prev_pos != position:
                pose_cell += 1
                cell_cost = self.costmap.get_cell(*position)
                if cell_cost is None:
                    cost += 999
                else:
                    cost += cell_cost
                    prev_pos = position

        return cost/pose_cell

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
            max_dist = 1e-5
            avg_dist = 0
            for global_pose in self.global_path.poses:
                dist = TrajectoryScorer.distance(pose, global_pose)
                min_dist = min(min_dist, dist)
                max_dist = max(max_dist, dist)
                avg_dist += dist

            avg_dist /= len(self.global_path.poses)
            cost.append(avg_dist)

        return sum(cost)#/len(trajectory.poses)

    @staticmethod
    def distance(pose1, pose2):
        # TODO (shrijit99): For grids with lower res, manhattan distance coverges faster. For higher res, use euclidean.
        return ((math.fabs(pose1.x - pose2.x))**0.5 + (math.fabs(pose1.y - pose2.y))**0.5) ** 2.0
        # return math.fabs(math.fabs(pose1.x - pose2.x) + math.fabs(pose2.y - pose1.y))

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
