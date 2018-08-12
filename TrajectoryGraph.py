import networkx as nx
from Trajectory import Trajectory
from Robot import Robot
import matplotlib.pyplot as plt
from networkx.drawing.nx_agraph import write_dot, graphviz_layout
from TrajectorySimulator import TrajectorySimulator
from SampleGenerator import SampleGenerator
import copy


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

        trajectories = self.generate_trajectory(robot)

        for trajectory in self.best_trajectory(trajectories, 4):
            new_robot = Robot(trajectory.poses[-1].x, trajectory.poses[-1].y, trajectory.poses[-1].theta,
                              trajectory.velocity.x, trajectory.velocity.theta, robot.max_accel[0], robot.max_accel[1])
            self.node_num += 1
            self.add_node(self.node_num, trajectory=trajectory)
            self.add_edge(parent_node_num, self.node_num, weight=trajectory.cost)
            self.add_trajectory(robot=new_robot, parent_node_num=self.node_num,
                             depth=(depth + 1))

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
        print(min_cost_path)
        for path in min_cost_path:
            final_path.append(self.nodes[path]["trajectory"])
        return final_path

    def generate_trajectory(self, robot):
        generator = SampleGenerator(robot, timestep=0.1, resolution_x=0.05, resolution_theta=0.05)
        vel_samples = generator.generate()
        simulator = TrajectorySimulator(time=4, timestep=0.1)
        trajectories = []
        for vel in vel_samples:
            trajectories.append(simulator.simulate_trajectory(robot, vel.x, vel.theta))
        return trajectories

    def draw_graph(self):
        plt.title('Trajectory Graph')
        pos = graphviz_layout(self, prog='dot')
        nx.draw(self, pos, with_labels=True, arrows=True)
        edge_labels = nx.get_edge_attributes(self, 'state')
        nx.draw_networkx_edge_labels(self, pos, labels=edge_labels)
        plt.show()

    def best_trajectory(self, trajectories, num_best_traj):
        # print("best vels:", trajectories[1].velocity.x, trajectories[2].velocity.theta)
        trajectory_list = []
        best_trajectories = []
        vis_costmap = copy.deepcopy(self.costmap)
        for trajectory in trajectories:
            cost = trajectory.score(self.costmap)
            trajectory_list.append((cost, trajectory))
            trajectory.visualize(vis_costmap, 10)
        vis_costmap.visualize()
        for i in range(0, num_best_traj):
            min_cost_trajectory = min(trajectory_list, key=lambda t: t[0])
            best_trajectories.append(min_cost_trajectory[1])
            trajectory_list.remove(min_cost_trajectory)
        return best_trajectories
