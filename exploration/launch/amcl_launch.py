import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import  RewrittenYaml
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():

    # map file path for simple_map
    map_file_path = os.path.join(
        get_package_share_directory('exploration'), 
        'map',
        'test_map.yaml'
    )

    amcl_file_path = os.path.join(
        get_package_share_directory('exploration'),
        'config', 
        'amcl_params.yaml', 
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=amcl_file_path,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )


    rviz_config_path = os.path.join(
        get_package_share_directory('exploration'),
        'config',
        'amcl_config.rviz')

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'yaml_filename': map_file_path}]
    )
    
    # start the node lifecyle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters = [{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters = [{'use_sim_time': False}],
        arguments=['-d', rviz_config_path]
    )

    pathplanner = Node(
        package = 'planning',
        executable = 'path_planner.py',
        name = 'path_planner',
        parameters = [{'use_sim_time': False}],
        output = 'screen',

        remappings = [
            ("/clicked_point", "/placeholder")
        ]
    )
    
    controller = Node(
        package = 'control',
        executable = 'controller.py',
        name = 'controller',
        output = 'screen',
        parameters = [{'use_sim_time': False}],
        remappings = [('/move_base_simple/goal', '/robot_path')]
    )

    amcl = Node(
        package = 'nav2_amcl',
        executable = 'amcl',
        name = 'amcl',
        output='screen',
        parameters = [configured_params]
    )

    
    reinit_global_loc = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            ' service call ',
            '/reinitialize_global_localization ',
            'std_srvs/srv/Empty ',
            '"{}"'
        ],
        shell=True
    )

    no_motion = ExecuteProcess(
            cmd=[
                FindExecutable(name='ros2'),
                ' service call ',
                '/request_nomotion_update ',
                'std_srvs/srv/Empty ',
                '"{}"'
            ],
            shell=True
        )


    localizer = Node(
        package='exploration',
        executable='Localization.py',
        name='localizer',
        parameters=[
            {'use_sim_time': False}
        ],
        output='screen'
    )

    return LaunchDescription([ 
        # gz_sim,
        map_server,
        amcl,
        lifecycle_manager, 
        rviz2,
        pathplanner,
        controller, 
        reinit_global_loc,
        no_motion,
        localizer, 
    ])
