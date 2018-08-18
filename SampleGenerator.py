import numpy as np
from Velocity import Velocity


class SampleGenerator:

    def __init__(self, robot, timestep, resolution_x, resolution_theta):
        self.current_vel_x = robot.velocity.x
        self.current_vel_theta = robot.velocity.theta
        self.max_accel_x = robot.max_accel[0]
        self.max_accel_theta = robot.max_accel[1]
        self.timestep = timestep
        self.resolution_x = resolution_x
        self.resolution_theta = resolution_theta

    def generate(self):
        x_vels = np.arange((self.current_vel_x - self.max_accel_x * self.timestep),\
                            (self.current_vel_x + self.max_accel_x * self.timestep + self.resolution_x),\
                            self.resolution_x)
        theta_vels = np.arange((self.current_vel_theta - self.max_accel_theta * self.timestep),\
                            (self.current_vel_theta + self.max_accel_theta * self.timestep + self.resolution_theta),\
                            self.resolution_theta)
        vel_samples = []
        for x_vel in x_vels:
            for theta_vel in theta_vels:
                vel_samples.append(Velocity(x_vel, theta_vel))
        np.random.shuffle(vel_samples)
        return vel_samples


