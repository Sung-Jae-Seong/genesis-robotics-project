import numpy as np
from jointGroup import JointGroup

class Arm(JointGroup):
    pass


class Hand(JointGroup):
    def __init__(self, model, joint_names):
        super().__init__(model, joint_names)
        self.is_open = False
        self.OPEN = 0.45
        self.CLOSE = -0.11
        self.open_joint = [self.OPEN, self.OPEN*0.1, -self.OPEN, -self.OPEN*0.1]
        self.close_joint = [self.CLOSE, self.CLOSE*0.1, -self.CLOSE, -self.CLOSE*0.1]

    def set_position(self, positions):
        super().set_position(positions)
        self.is_open = not np.allclose(positions, self.close_joint)

    def control_position(self, positions):
        super().control_position(positions)
        self.is_open = not np.allclose(positions, self.close_joint)

    def open(self):
        self.control_position(self.open_joint)

    def close(self):
        self.control_position(self.close_joint)

    def is_opened(self):
        return self.is_open
    
    def get_OPEN_value(self):
        return self.OPEN
    
    def get_CLOSE_value(self):
        return self.CLOSE  
    
    def set_OPEN_VALUE(self, value):
        self.OPEN = value
        self.open_joint = [self.OPEN, self.OPEN*0.1, -self.OPEN, -self.OPEN*0.1]

    def set_CLOSE_VALUE(self, value):
        self.CLOSE = value
        self.close_joint = [self.CLOSE, self.CLOSE*0.1, -self.CLOSE, -self.CLOSE*0.1]


class RobotEntity:
    def __init__(self, model):
        self.model = model
        self.Arm = None
        self.Hand = None

    def init_arm(self, joint_names):
        self.Arm = Arm(self.model, joint_names)

    def init_hand(self, joint_names):
        self.Hand = Hand(self.model, joint_names)
