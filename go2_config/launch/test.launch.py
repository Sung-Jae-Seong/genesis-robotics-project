from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Quadruped Controller Node
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
    )
    return LaunchDescription([
        quadruped_controller_node,
    ])
