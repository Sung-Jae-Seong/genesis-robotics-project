/*
Copyright (c) 2019-2020, Juan Miguel Jimeno

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <quadruped_controller.h>
#include <map>
#include <iostream>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time) {
  return time.nanoseconds() / 1000ul;
}

QuadrupedController::QuadrupedController():
    Node("quadruped_controller_node", rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(clock_.now())),
    kinematics_(base_)
{
    speed = 0.5;
    turn = 1.0;
    std::string knee_orientation;
    std::string urdf = R"(<?xml version="1.0" ?>
<robot name="go2">
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.2 0.2 0.2 0.0"/>
  </material>
  <material name="silver">
    <color rgba="0.9137254901960784 0.9137254901960784 0.8470588235294118 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.4235294117647059 0.0392156862745098 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.8705882352941177 0.8117647058823529 0.7647058823529411 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <!-- <xacro:include filename="$(find go2_description)/xacro/transmission.xacro"/> -->
  <!-- <xacro:include filename="$(find go2_description)/xacro/stairs.xacro"/> -->
  <!-- <xacro:include filename="$(find go2_description)/xacro/gazebo.xacro"/> -->
  <!-- <xacro:include filename="$(find go2_gazebo)/launch/stairs.urdf.xacro"/> -->
  <!-- <xacro:stairs stairs="15" xpos="0" ypos="0" zpos="0" /> -->
  <!-- Rotor related joint and link is only for demonstrate location. -->
  <!-- Actually, the rotor will rotate and the joint is not fixed. Reduction ratio should be considered. -->
  <link name="base_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>
  <joint name="floating_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_link"/>
    <child link="trunk"/>
  </joint>
  <link name="trunk">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/trunk.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.3762 0.0935 0.114"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.021112 0.0 -0.005366"/>
      <mass value="6.921"/>
      <inertia ixx="0.02448" ixy="0.00012166" ixz="0.0014849" iyy="0.098077" iyz="-3.12e-05" izz="0.107"/>
    </inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="trunk"/>
    <child link="imu_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="imu_link">
    <inertial>
      <mass value="0.001"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.001 0.001 0.001"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size=".001 .001 .001"/>
      </geometry>
    </collision>
  </link>
  <!--
    <joint name="load_joint" type="fixed">
        <parent link="trunk"/>
        <child link="load_link"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>

    <link name="load_link">
        <inertial>
            <mass value="5"/>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.2"/>
            <geometry>
                <box size="0.5 0.3 0.2"/>
            </geometry>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <box size=".001 .001 .001"/>
            </geometry>
        </collision>
    </link>
  -->
  <joint name="rf_hip_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.1934 -0.0465 0"/>
    <parent link="trunk"/>
    <child link="rf_hip_link"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.0472" upper="1.0472" velocity="30.1"/>
  </joint>
  <link name="rf_hip_link">
    <visual>
      <origin rpy="3.141592653589793 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/hip.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.0955 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.046"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0054 -0.00194 -0.000105"/>
      <mass value="0.678"/>
      <inertia ixx="0.00048" ixy="3.01e-06" ixz="1.11e-06" iyy="0.000884" iyz="1.42e-06" izz="0.000596"/>
    </inertial>
  </link>
  <joint name="rf_upper_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 -0.0955 0"/>
    <parent link="rf_hip_link"/>
    <child link="rf_upper_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.5708" upper="3.4907" velocity="30.1"/>
  </joint>
  <link name="rf_upper_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/thigh_mirror.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.0245 0.034"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00374 0.0223 -0.0327"/>
      <mass value="1.152"/>
      <inertia ixx="0.00584" ixy="-8.72e-05" ixz="-0.000289" iyy="0.0058" iyz="-0.000808" izz="0.00103"/>
    </inertial>
  </link>
  <joint name="rf_lower_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="rf_upper_leg_link"/>
    <child link="rf_lower_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="35.55" lower="-2.7227" upper="-0.83776" velocity="20.06"/>
  </joint>
  <link name="rf_lower_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/calf.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.016 0.016"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00548 -0.000975 -0.115"/>
      <mass value="0.154"/>
      <inertia ixx="0.00108" ixy="3.4e-07" ixz="1.72e-05" iyy="0.0011" iyz="8.28e-06" izz="3.29e-05"/>
    </inertial>
  </link>
  <joint name="rf_foot_joint" type="fixed">
    0
            
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="rf_lower_leg_link"/>
    <child link="rf_foot_link"/>
  </joint>
  <link name="rf_foot_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.06"/>
      <inertia ixx="9.600000000000001e-06" ixy="0.0" ixz="0.0" iyy="9.600000000000001e-06" iyz="0.0" izz="9.600000000000001e-06"/>
    </inertial>
  </link>
  <ros2_control name="rf" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="rf_hip_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rf_upper_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rf_lower_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <!-- <xacro:leg_transmission name="${name}"/> -->
  <joint name="lf_hip_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0.1934 0.0465 0"/>
    <parent link="trunk"/>
    <child link="lf_hip_link"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.0472" upper="1.0472" velocity="30.1"/>
  </joint>
  <link name="lf_hip_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/hip.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0.0955 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.046"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.0054 0.00194 -0.000105"/>
      <mass value="0.678"/>
      <inertia ixx="0.00048" ixy="-3.01e-06" ixz="1.11e-06" iyy="0.000884" iyz="-1.42e-06" izz="0.000596"/>
    </inertial>
  </link>
  <joint name="lf_upper_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0.0955 0"/>
    <parent link="lf_hip_link"/>
    <child link="lf_upper_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.5708" upper="3.4907" velocity="30.1"/>
  </joint>
  <link name="lf_upper_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/thigh.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.0245 0.034"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00374 -0.0223 -0.0327"/>
      <mass value="1.152"/>
      <inertia ixx="0.00584" ixy="8.72e-05" ixz="-0.000289" iyy="0.0058" iyz="0.000808" izz="0.00103"/>
    </inertial>
  </link>
  <joint name="lf_lower_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="lf_upper_leg_link"/>
    <child link="lf_lower_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="35.55" lower="-2.7227" upper="-0.83776" velocity="20.06"/>
  </joint>
  <link name="lf_lower_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/calf.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.016 0.016"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00548 -0.000975 -0.115"/>
      <mass value="0.154"/>
      <inertia ixx="0.00108" ixy="3.4e-07" ixz="1.72e-05" iyy="0.0011" iyz="8.28e-06" izz="3.29e-05"/>
    </inertial>
  </link>
  <joint name="lf_foot_joint" type="fixed">
    0
            
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="lf_lower_leg_link"/>
    <child link="lf_foot_link"/>
  </joint>
  <link name="lf_foot_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.06"/>
      <inertia ixx="9.600000000000001e-06" ixy="0.0" ixz="0.0" iyy="9.600000000000001e-06" iyz="0.0" izz="9.600000000000001e-06"/>
    </inertial>
  </link>
  <ros2_control name="lf" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="lf_hip_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="lf_upper_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="lf_lower_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <!-- <xacro:leg_transmission name="${name}"/> -->
  <joint name="rh_hip_joint" type="revolute">
    <origin rpy="0 0 0" xyz="-0.1934 -0.0465 0"/>
    <parent link="trunk"/>
    <child link="rh_hip_link"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.0472" upper="1.0472" velocity="30.1"/>
  </joint>
  <link name="rh_hip_link">
    <visual>
      <origin rpy="3.141592653589793 3.141592653589793 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/hip.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 -0.0955 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.046"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0054 -0.00194 -0.000105"/>
      <mass value="0.678"/>
      <inertia ixx="0.00048" ixy="-3.01e-06" ixz="-1.11e-06" iyy="0.000884" iyz="1.42e-06" izz="0.000596"/>
    </inertial>
  </link>
  <joint name="rh_upper_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 -0.0955 0"/>
    <parent link="rh_hip_link"/>
    <child link="rh_upper_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.5708" upper="3.4907" velocity="30.1"/>
  </joint>
  <link name="rh_upper_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/thigh_mirror.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.0245 0.034"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00374 0.0223 -0.0327"/>
      <mass value="1.152"/>
      <inertia ixx="0.00584" ixy="-8.72e-05" ixz="-0.000289" iyy="0.0058" iyz="-0.000808" izz="0.00103"/>
    </inertial>
  </link>
  <joint name="rh_lower_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="rh_upper_leg_link"/>
    <child link="rh_lower_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="35.55" lower="-2.7227" upper="-0.83776" velocity="20.06"/>
  </joint>
  <link name="rh_lower_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/calf.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.016 0.016"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00548 -0.000975 -0.115"/>
      <mass value="0.154"/>
      <inertia ixx="0.00108" ixy="3.4e-07" ixz="1.72e-05" iyy="0.0011" iyz="8.28e-06" izz="3.29e-05"/>
    </inertial>
  </link>
  <joint name="rh_foot_joint" type="fixed">
    0
            
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="rh_lower_leg_link"/>
    <child link="rh_foot_link"/>
  </joint>
  <link name="rh_foot_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.06"/>
      <inertia ixx="9.600000000000001e-06" ixy="0.0" ixz="0.0" iyy="9.600000000000001e-06" iyz="0.0" izz="9.600000000000001e-06"/>
    </inertial>
  </link>
  <ros2_control name="rh" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="rh_hip_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rh_upper_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="rh_lower_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <!-- <xacro:leg_transmission name="${name}"/> -->
  <joint name="lh_hip_joint" type="revolute">
    <origin rpy="0 0 0" xyz="-0.1934 0.0465 0"/>
    <parent link="trunk"/>
    <child link="lh_hip_link"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.0472" upper="1.0472" velocity="30.1"/>
  </joint>
  <link name="lh_hip_link">
    <visual>
      <origin rpy="0 3.141592653589793 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/hip.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0.0955 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.046"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0054 0.00194 -0.000105"/>
      <mass value="0.678"/>
      <inertia ixx="0.00048" ixy="3.01e-06" ixz="-1.11e-06" iyy="0.000884" iyz="-1.42e-06" izz="0.000596"/>
    </inertial>
  </link>
  <joint name="lh_upper_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0.0955 0"/>
    <parent link="lh_hip_link"/>
    <child link="lh_upper_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="23.7" lower="-1.5708" upper="3.4907" velocity="30.1"/>
  </joint>
  <link name="lh_upper_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/thigh.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.0245 0.034"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="-0.00374 -0.0223 -0.0327"/>
      <mass value="1.152"/>
      <inertia ixx="0.00584" ixy="8.72e-05" ixz="-0.000289" iyy="0.0058" iyz="0.000808" izz="0.00103"/>
    </inertial>
  </link>
  <joint name="lh_lower_leg_joint" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="lh_upper_leg_link"/>
    <child link="lh_lower_leg_link"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.01" friction="0.2"/>
    <limit effort="35.55" lower="-2.7227" upper="-0.83776" velocity="20.06"/>
  </joint>
  <link name="lh_lower_leg_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="file:///home/sjs/genesis-robotics-project/quadruped_controller/install/go2_description/share/go2_description/meshes/calf.dae" scale="1 1 1"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
      <geometry>
        <box size="0.213 0.016 0.016"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy="0 0 0" xyz="0.00548 -0.000975 -0.115"/>
      <mass value="0.154"/>
      <inertia ixx="0.00108" ixy="3.4e-07" ixz="1.72e-05" iyy="0.0011" iyz="8.28e-06" izz="3.29e-05"/>
    </inertial>
  </link>
  <joint name="lh_foot_joint" type="fixed">
    0
            
    <origin rpy="0 0 0" xyz="0 0 -0.213"/>
    <parent link="lh_lower_leg_link"/>
    <child link="lh_foot_link"/>
  </joint>
  <link name="lh_foot_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <!-- <material name="orange"/> -->
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.06"/>
      <inertia ixx="9.600000000000001e-06" ixy="0.0" ixz="0.0" iyy="9.600000000000001e-06" iyz="0.0" izz="9.600000000000001e-06"/>
    </inertial>
  </link>
  <ros2_control name="lh" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="lh_hip_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="lh_upper_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="lh_lower_leg_joint">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <!-- <xacro:leg_transmission name="${name}"/> -->
</robot>
)";

    double loop_rate = 200.0;

    gait_config_.pantograph_leg = false;
    gait_config_.max_linear_velocity_x = 0.3;
    gait_config_.max_linear_velocity_y = 0.25;
    gait_config_.max_angular_velocity_z = 0.5;
    gait_config_.com_x_translation = 0.0;
    gait_config_.swing_height = 0.04;
    gait_config_.stance_depth = 0.01;
    gait_config_.stance_duration = 0.25;
    gait_config_.nominal_height = 0.225;
    knee_orientation = ">>";

    publish_joint_states_ = true;
    publish_joint_control_ = true;
    std::string joint_control_topic = "joint_group_effort_controller/joint_trajectory";

    if (publish_joint_control_) {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    gait_config_.knee_orientation = knee_orientation.c_str();

    base_.setGaitConfig(gait_config_);

    std::vector<std::vector<std::string>> links_map = {
        {"lf_hip_link", "lf_upper_leg_link", "lf_lower_leg_link", "lf_foot_link"},
        {"rf_hip_link", "rf_upper_leg_link", "rf_lower_leg_link", "rf_foot_link"},
        {"lh_hip_link", "lh_upper_leg_link", "lh_lower_leg_link", "lh_foot_link"},
        {"rh_hip_link", "rh_upper_leg_link", "rh_lower_leg_link", "rh_foot_link"}
    };
    champ::URDF::loadFromString(base_, links_map, urdf);

    std::map<std::string, std::vector<std::string>> joints_map = {
        {"left_front", {"lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", "lf_foot_joint"}},
        {"right_front", {"rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", "rf_foot_joint"}},
        {"left_hind", {"lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", "lh_foot_joint"}},
        {"right_hind", {"rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint", "rh_foot_joint"}}
    };
    joint_names_ = champ::URDF::getJointNames(joints_map);

    std::chrono::milliseconds period(static_cast<int>(1000/loop_rate));

    loop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&QuadrupedController::controlLoop_, this)
    );
    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_() {
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    req_vel_.linear.x = 1*speed;
    req_vel_.linear.y = 0*speed;
    req_vel_.angular.z = 0*turn;

    publishJoints_(target_joint_positions);
}

void QuadrupedController::publishJoints_(float target_joints[12]) {
    if (publish_joint_control_) {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();

        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;

        point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
        point.positions.resize(12);
        point.positions.assign(target_joints, target_joints + 12);

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }
}
