import networkx as nx
from Trajectory import Trajectory
from Robot import Robot
import matplotlib.pyplot as plt
from networkx.drawing.nx_agraph import graphviz_layout
from TrajectoryScorer import TrajectoryScorer
import os
import math


class TrajectoryGraph(nx.DiGraph):
    def __init__(self, costmap, node_num=0, depth=1):
        self.node_num = node_num
        self.costmap = costmap
        self.depth = depth
        super().__init__()

    def build_graph(self, robot):
        root_trajectory = Trajectory(poses=None, velocity=robot.velocity)
        self.add_node(self.node_num, trajectory=root_trajectory)
        self.add_trajectory(robot=robot, parent_node_num=0, depth=0)
        return self

    def add_trajectory(self, robot, parent_node_num, depth):
        if depth == self.depth:
            return

        trajectories = Trajectory.generate_trajectories(robot, self.costmap, depth)

        scorer = TrajectoryScorer(self.costmap)
        best_trajectories = scorer.best_trajectory(trajectories, (int(os.environ.get("BEST_TRAJECTORIES"))), depth)
        for trajectory in best_trajectories:
            new_robot = Robot(trajectory.poses[-1].x, trajectory.poses[-1].y, trajectory.poses[-1].theta,
                              trajectory.velocity.x, trajectory.velocity.theta, robot.max_accel[0], robot.max_accel[1])
            self.node_num += 1
            self.add_node(self.node_num, trajectory=trajectory)
            self.add_edge(parent_node_num, self.node_num, weight=trajectory.cost)
            self.add_trajectory(robot=new_robot, parent_node_num=self.node_num, depth=(depth + 1))

    def draw_graph(self):
        plt.title('Trajectory Graph')
        pos = graphviz_layout(self, prog='dot')
        nx.draw(self, pos, with_labels=True, arrows=True)
        edge_labels = nx.get_edge_attributes(self, 'state')
        nx.draw_networkx_edge_labels(self, pos, labels=edge_labels)
        plt.show()

    def final_trajectory(self):
        leaves = [x for x in self.nodes() if self.out_degree(x) == 0 and self.in_degree(x) == 1]
        trajectory_costs = []
        for leaf in leaves:
            cost = nx.dijkstra_path_length(self, 0, leaf)
            trajectory_costs.append((leaf, cost))
        minimum_cost_leaf = min(trajectory_costs, key=lambda t: t[1])
        min_cost_path = nx.dijkstra_path(self, 0, minimum_cost_leaf[0])
        del min_cost_path[0]
        final_path = []
        for path in min_cost_path:
            final_path.append(self.nodes[path]["trajectory"])
        return final_path