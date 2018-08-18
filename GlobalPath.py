import numpy as np
from Pose import Pose


class GlobalPath:

    def __init__(self, poses):
        self.poses = np.array(poses)
