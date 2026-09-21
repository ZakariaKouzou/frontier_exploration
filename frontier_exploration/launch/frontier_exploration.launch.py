import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('frontier_exploration')
    default_params = os.path.join(pkg_share, 'config', 'frontier_exploration_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the frontier_exploration params file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    detector_node = Node(
        package='frontier_exploration',
        executable='frontier_detector_node',
        name='frontier_detector_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    selector_node = Node(
        package='frontier_exploration',
        executable='frontier_goal_selector_node',
        name='frontier_goal_selector',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        detector_node,
        selector_node,
    ])