from Costmap import Costmap
from Robot import Robot
from TrajectoryGraph import TrajectoryGraph

if __name__ == "__main__":
    costmap = Costmap(750, 750, 0.05)
    robot = Robot(x=0, y=0, theta=0, current_vel_x=0, current_vel_theta=0, max_accel_x=4, max_accel_theta=0.3)

    trajectory_graph = TrajectoryGraph(costmap)
    trajectory_graph.build_graph(robot)
    trajectory_graph.draw_graph()
    final_path = trajectory_graph.final_trajectory()
    i = 0
    for trajectory in final_path:
        # print(trajectory)
        # for pose in trajectory.poses:
            # print(pose.x, pose.y, pose.theta)
        trajectory.visualize(costmap, i)
        i += 1
    costmap.visualize()





