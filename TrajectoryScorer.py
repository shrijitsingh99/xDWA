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
        for x in range(0, 130):
            poses.append(Pose(x/3, x))
        for x in range(43, 200):
            poses.append(Pose(x, 120+x/5))

        # for x in range(costmap.x):
        #     poses.append(Pose(x, x))

        self.global_path = GlobalPath(poses)

    def trajectory_cost(self, trajectory):
        costmap_cost = self.costmap_cost(trajectory) + 1
        obstacle_dist_cost = self.obstacle_dist_cost(trajectory) + 1
        global_path_cost = self.global_path_cost(trajectory) + 1
        twirling_cost = self.twirling_cost(trajectory) + 1
        goal_cost = self.goal_cost(trajectory) + 1
        cost = 100*(100*costmap_cost + 5 * global_path_cost+ goal_cost)
        trajectory.cost = cost
        return cost

    def costmap_cost(self, trajectory):
        cost = 0
        # prev_pos = (0, 0)
        # TODO (squadrick): prev_pos should be Robot's starting x and y
        for pose in trajectory.poses:
            position = (math.floor(pose.x / self.costmap.resolution), math.floor(pose.y / self.costmap.resolution))
            # if prev_pos != position:
            cell_cost = self.costmap.get_cell(*position)
            if cell_cost is None:
                cost += 999
            else:
                cost += cell_cost
                # prev_pos = position
        return cost

    def obstacle_dist_cost(self, trajectory):
        mid = len(trajectory.poses)
        mid_pose = trajectory.poses[int(mid / 2)]
        dist = []
        for obstacle in self.costmap.obstacles:
            obs_dist = math.sqrt(math.fabs(((mid_pose.x - obstacle[0])**2) - ((mid_pose.y - obstacle[1])**2)))
            if obs_dist <= 15:
                dist.append(obs_dist)
        cost = 0
        if len(dist) > 0:
            cost = sum(dist)/len(dist)
        cost += 1

        return 1/cost

    def global_path_cost(self, trajectory):
        cost = []
        avg_dist = 0
        for pose in trajectory.poses:
            min_dist = 1e10
            for global_pose in self.global_path.poses:
                min_dist = min(min_dist, TrajectoryScorer.distance(pose, global_pose))
            avg_dist += min_dist**2.5
        avg_dist /= len(self.global_path.poses)
        cost.append(avg_dist)
        return sum(cost)/len(trajectory.poses)

    def twirling_cost(self, trajectory):
        return math.fabs(trajectory.velocity.theta)

    def goal_cost(self, trajectory):
        return TrajectoryScorer.distance(trajectory.poses[-1], Pose(200, 145, 0))

    @staticmethod
    def distance(pose1, pose2):
        # TODO (shrijitsingh99): For grids with lower res, manhattan distance coverges faster. For higher res, use euclidean.
        return ((math.fabs(pose1.x - pose2.x))**2 + (math.fabs(pose1.y - pose2.y))**2) ** 0.5
        # return math.fabs(math.fabs(pose1.x - pose2.x) + math.fabs(pose2.y - pose1.y))

    def best_trajectory(self, trajectories, num_best_traj, depth):
        trajectory_list = []
        best_trajectories = []
        cost_diff = []
        for trajectory in trajectories:
            cost = trajectory.score(self.costmap)
            
            trajectory_list.append((cost, trajectory))

        if int(os.environ.get("DISPLAY_PATH_SEARCH")):
            vis_costmap = copy.deepcopy(self.costmap)
            for trajectory in trajectories:
                trajectory.visualize(vis_costmap, (depth + 1)*50)
            vis_costmap.display(close=True)
        prev_min_cost = -1

        for i in range(0, len(trajectories)):
            if num_best_traj == 0:
                break
            min_cost_trajectory = min(trajectory_list, key=lambda t: t[0])
            # if math.fabs(prev_min_cost - min_cost_trajectory[0]) < 1000:
            #     trajectory_list.remove(min_cost_trajectory)
            #     continue

            best_trajectories.append(min_cost_trajectory[1])
            num_best_traj -=1
            prev_min_cost = min_cost_trajectory[0]
            trajectory_list.remove(min_cost_trajectory)

        return best_trajectories
