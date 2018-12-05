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
    robot = Robot(x=10, y=10, theta=0, current_vel_x=1, current_vel_theta=0, max_accel_x=0.5, max_accel_theta=2)
        #
        # for x in range(0, 130):
        #     self.map[int(x/3)][x] = 100
        # for x in range(43, 150):
        #     self.map[x][int(120+x/5)] = 100
    from Trajectory import Trajectory
    from Velocity import Velocity
    from TrajectoryScorer import TrajectoryScorer

    potential = Costmap(230, 230, 1)

    for x in range(0, 230):
        for y in range(0, 230):
            potential.map[x][y] = 0

    for x in range(0, 230, 4):
        for y in range(0, 230, 4):
            t = Trajectory([Pose(x, y)], Velocity(0, 0))
            potential.map[x][y] = t.score(costmap)
    potential.display()
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    x = [i for i in range(230)]
    y = [i for i in range(230)]
    X, Y = np.meshgrid(x, y)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.view_init(75, 270)
    ax.plot_surface(X, Y, potential.map, rstride=1, cstride=1,
                    cmap='viridis')

    for i in range(int(os.environ.get("NUMBER_OF_STEPS"))):
        print("Step: ", i)
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





