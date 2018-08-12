from Costmap import Costmap
from Robot import Robot
from TrajectoryGraph import TrajectoryGraph
import math
import copy

if __name__ == "__main__":
    costmap = Costmap(200, 200, 1)
    visual_costmap = copy.deepcopy(costmap)
    robot = Robot(x=0, y=0, theta=0, current_vel_x=10, current_vel_theta=0, max_accel_x=4, max_accel_theta=1)

    for i in range(0, 3):
        trajectory_graph = TrajectoryGraph(costmap, depth=1)
        trajectory_graph.build_graph(robot)
        trajectory_graph.draw_graph()
        final_path = trajectory_graph.final_trajectory()
        i = 0
        for trajectory in final_path:
            # print(trajectory)
            # for pose in trajectory.poses:
                # print(pose.x, pose.y, pose.theta)
            print(trajectory.velocity.x, trajectory.velocity.theta)
            trajectory.visualize(visual_costmap, i)
            i += 1
        robot.pose = final_path[0].poses[-1]
        robot.velocity = final_path[0].velocity
        for x in range(math.floor(robot.pose.x), math.floor(robot.pose.x) + 5):
            for y in range(math.floor(robot.pose.y), math.floor(robot.pose.y) + 5):
                visual_costmap.map[x][y] = 100
    visual_costmap.visualize()





