import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='spice',
            executable='SwarmManager.py',
            name='SwarmManager'
        ),

        Node(
            package='spice',
            executable='work_cell_allocator',
            name='work_cell_allocator'
        ),

        Node(
            package='spice',
            executable='task_allocator',
            name='task_allocator'
        ),

        Node(
            package='spice',
            executable='work_cell_simulator',
            name='work_cell_simulator'
        ),

        Node(
            package='spice',
            executable='central_path_planner',
            name='central_path_planner'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('spice_nav'),
                    'launch/global_map_server.launch.py'
                )
            ),
        )
    ])