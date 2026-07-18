"""Launch both custom motion Action Servers and the square client."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the custom square-motion launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_parameters = [{
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the ROS simulation clock',
        ),
        Node(
            package='turtlebot4_controller',
            executable='drive_distance_action_server',
            name='drive_distance_action_server',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='turtlebot4_controller',
            executable='rotate_angle_action_server',
            name='rotate_angle_action_server',
            output='screen',
            parameters=common_parameters,
        ),
        Node(
            package='turtlebot4_controller',
            executable='move_square',
            name='move_square',
            output='screen',
            parameters=common_parameters,
        ),
    ])
