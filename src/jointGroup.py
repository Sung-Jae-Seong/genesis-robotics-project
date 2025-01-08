class JointGroup:
    def __init__(self, model, joint_names):
        self.model = model
        self.joint_indices = [self.model.get_joint(name).dof_idx_local for name in joint_names]
        self.joint_num = len(self.joint_indices)

    def set_joint_kp(self, gains):
        self.model.set_dofs_kp(gains, self.joint_indices)

    def set_joint_vel(self, vels):
        self.model.set_dofs_kv(vels, self.joint_indices)

    def set_force_range(self, lower, upper):
        self.model.set_dofs_force_range(lower, upper, self.joint_indices)

    def set_position(self, positions):
        self.model.set_dofs_position(positions, self.joint_indices)

    def control_position(self, positions):
        self.model.control_dofs_position(positions, self.joint_indices)