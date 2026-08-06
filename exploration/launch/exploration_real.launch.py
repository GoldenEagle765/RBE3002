import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():   

    rviz_config_dir = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'exploration_sim_config.rviz')

    slam_config_dir = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'slam_params_real.yaml'
    )

    rviz2 =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')
    
    filter = Node(
        package="laser_filters",    
        executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("exploration"),
                    "config",
                    "bin_filter.yaml",
                ])
            ],
        remappings = [
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
        )
    

    controller = Node(
        package = 'control',
        executable = 'controller.py',
        name = 'controller',
        parameters = [{'use_sim_time': False}],
        output='screen'
    )

    slam_node_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'slam_launch.py')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_node_launch_path),
        launch_arguments={'slam_params_file': slam_config_dir}.items()
    )


    path_planner_node = Node(
        package = 'planning',
        executable = 'path_planner.py',
        name = 'path_planner',
        parameters = [{'use_sim_time': False}],
        output='screen'
    )

    frontier_explorer_node = Node(
        package = 'exploration',
        executable = 'frontier_explorer.py',
        name = 'frontier_explorer',
        parameters = [{'use_sim_time': False}],
        output = 'screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=False),
        rviz2,
        controller,
        slam_launch,
        path_planner_node,
        frontier_explorer_node,
        filter,
    ])
