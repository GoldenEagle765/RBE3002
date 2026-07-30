import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config_dir = os.path.join(
        get_package_share_directory('control'),
        'rviz',
        'control_pkg.rviz'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'map', 'odom'
        ],
        output='screen'
    )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        parameters=[{'use_sim_time': False}],
        output='screen',
        remappings=[
            ('/move_base_simple/goal', '/placeholder'),
        ]
        
    )

    return LaunchDescription([
        rviz2,
        static_transform,
        controller
    ])
