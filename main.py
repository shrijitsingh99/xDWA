from Costmap import Costmap
from Robot import Robot
from TrajectoryGraph import TrajectoryGraph
import math
import copy
import configuration
from GlobalPath import GlobalPath
from Pose import Pose
import os

if __name__ == "__main__":
    costmap = Costmap(230, 230, 1)
    visual_costmap = copy.deepcopy(costmap)
    robot = Robot(x=0, y=0, theta=0, current_vel_x=10, current_vel_theta=0, max_accel_x=3, max_accel_theta=1)

    for i in range(int(os.environ.get("NUMBER_OF_STEPS"))):
        trajectory_graph = TrajectoryGraph(costmap, depth=int(os.environ.get("SEARCH_DEPTH")))
        trajectory_graph.build_graph(robot)

        if int(os.environ.get("DISPLAY_GRAPH")):
            trajectory_graph.draw_graph()

        final_path = trajectory_graph.final_trajectory()
        i = 0
        for trajectory in final_path:
            trajectory.visualize(visual_costmap, i)
            i += 1
        robot.pose = final_path[0].poses[-1]
        robot.velocity = final_path[0].velocity

        for x in range(int(math.floor(robot.pose.x)), int(math.floor(robot.pose.x) + 5)):
            for y in range(int(math.floor(robot.pose.y)), int(math.floor(robot.pose.y) + 5)):
                visual_costmap.set_cell(x, y, 100)
    visual_costmap.display()


#
# def create_global_path(costmap):
#     poses = []
#     for x in range(costmap.x):
#         poses.append(Pose(x, x))
#     global_path = GlobalPath(poses)
#
#     return global_path





