class ManipulatorEntity:
    def __init__(self, model, arm_joint_names, gripper_joint_names):
        self.model = model
        self.arm_indices = [self.model.get_joint(name).dof_idx_local for name in arm_joint_names]
        self.arm_num = len(self.arm_indices)
        self.gripper_indices = [self.model.get_joint(name).dof_idx_local for name in gripper_joint_names]
        self.gripper_num = len(self.gripper_indices)
        #gripper variables
        self.is_open = False
        self.OPEN = 0.45
        self.CLOSE = -0.11
        self.open_gripper_joint = [self.OPEN, self.OPEN*0.1, -self.OPEN, -self.OPEN*0.1]
        self.close_gripper_joint = [self.CLOSE, self.CLOSE*0.1, -self.CLOSE, -self.CLOSE*0.1]

    def set_joint_kp(self, gains, joint_indices):
        self.model.set_dofs_kp(gains, joint_indices)

    def set_joint_vel(self, vels, joint_indices):
        self.model.set_dofs_kv(vels, joint_indices)

    def set_force_range(self, lower, upper, joint_indices):
        self.model.set_dofs_force_range(lower, upper, joint_indices)

    def set_position(self, positions, joint_indices):
        self.model.set_dofs_position(positions, joint_indices)

    def control_position(self, positions, joint_indices):
        self.model.control_dofs_position(positions, joint_indices)
    
    def open_gripper(self):
        self.control_position(self.open_gripper_joint, self.gripper_indices)

    def close_gripper(self):
        self.control_position(self.close_gripper_joint, self.gripper_indices)

    def is_opened(self):
        return self.is_open
    
    def get_OPEN_value(self):
        return self.OPEN
    
    def get_CLOSE_value(self):
        return self.CLOSE  
    
    def set_OPEN_VALUE(self, value):
        self.OPEN = value
        self.open_gripper_joint = [self.OPEN, self.OPEN*0.1, -self.OPEN, -self.OPEN*0.1]

    def set_CLOSE_VALUE(self, value):
        self.CLOSE = value
        self.close_gripper_joint = [self.CLOSE, self.CLOSE*0.1, -self.CLOSE, -self.CLOSE*0.1]
