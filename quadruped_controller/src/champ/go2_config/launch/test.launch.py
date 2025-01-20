import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    config_pkg_share = os.path.join(
        get_package_share_directory("go2_config")
    )
    descr_pkg_share = os.path.join(
        get_package_share_directory("go2_description")
    )

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")

    # LaunchConfiguration for flexibility
    joints_map_path = LaunchConfiguration("joints_map_path", default=joints_config)
    links_map_path = LaunchConfiguration("links_map_path", default=links_config)
    gait_config_path = LaunchConfiguration("gait_config_path", default=gait_config)


    # Quadruped Controller Node
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"gazebo": False},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": Command(["xacro ", default_model_path])},
            joints_map_path,
            links_map_path,
            gait_config_path,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    return LaunchDescription([
        quadruped_controller_node,
    ])
