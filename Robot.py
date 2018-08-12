from Pose import Pose
from Velocity import Velocity

class Robot:
    def __init__(self, x, y, theta, current_vel_x, current_vel_theta, max_accel_x, max_accel_theta):
        self.pose = Pose(x, y, theta)
        self.velocity = Velocity(current_vel_x, current_vel_theta)
        self.max_accel = (max_accel_x, max_accel_theta)
