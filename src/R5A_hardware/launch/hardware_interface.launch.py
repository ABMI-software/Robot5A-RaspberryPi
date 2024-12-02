from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='R5A_hardware',
            executable='slush_hardware_interface',
            name='hardware_interface',
            output='screen',
            parameters=['config/ros2_control_config.yaml']
        )
    ])
