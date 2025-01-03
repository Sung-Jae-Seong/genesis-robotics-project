import numpy as np
from jointGroup import JointGroup

class Arm(JointGroup):
    pass


class Hand(JointGroup):
    def __init__(self, model, joint_names):
        super().__init__(model, joint_names)
        self.is_open = False

    def set_position(self, positions):
        super().set_position(positions)
        self.is_open = not np.allclose(positions, [0.0, 0.0])

    def control_position(self, positions):
        super().control_position(positions)
        self.is_open = not np.allclose(positions, [0.0, 0.0])

    def open(self):
        self.control_position([0.7, 0.7])

    def close(self):
        self.control_position([0.0, 0.0])


class RobotEntity:
    def __init__(self, model):
        self.model = model
        self.Arm = None
        self.Hand = None

    def init_arm(self, joint_names):
        self.Arm = Arm(self.model, joint_names)

    def init_hand(self, joint_names):
        self.Hand = Hand(self.model, joint_names)
