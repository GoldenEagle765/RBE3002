import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():   

    small_field_launch_path = os.path.join(get_package_share_directory('simulation'), 'launch', 'small_field.launch.py')

    small_field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(small_field_launch_path),
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'exploration_sim_config.rviz')

    slam_config_dir = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'slam_params.yaml'
    )

    rviz2 =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    static_transform = Node(
        package="tf2_ros", 
        executable="static_transform_publisher",
        arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"] 
    )

    controller = Node(
        package = 'control',
        executable = 'controller.py',
        name = 'controller',
        parameters = [{'use_sim_time': True}],
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
        parameters = [{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        small_field_launch,
        rviz2,
        static_transform,
        controller,
        slam_launch,
        path_planner_node
    ])