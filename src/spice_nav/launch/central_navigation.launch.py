# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

DEFAULT_MAP_NAME = 'C4.yaml' # change to the name.yaml of the default map here

def generate_launch_description():

    map_name = LaunchConfiguration('map')
    map_path = LaunchConfiguration('map_path')

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'), 'maps', map_name]
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'launch', 'nav_bringup.launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'config', 'navigation.yaml']
    )

    DeclareLaunchArgument(
        name = 'nr',
        description='The PolyBot\'s number'
    ),

    use_namespace = 'true'
    namespace = ['polybot', LaunchConfiguration("nr")]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value = DEFAULT_MAP_NAME,
            description='Map name.yaml'
        ),

        DeclareLaunchArgument(
            name='map_path',
            default_value = default_map_path,
            description='Map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map_path': map_path,
                'use_sim_time': LaunchConfiguration("sim"),
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_composition': 'True',
                'params_file': nav2_config_path,
                'namespace' : namespace,
                'autostart' : 'True'
            }.items()
        ),
        Node(
            package='spice',
            executable='robot_tf_pose',
            name='robot_pose_relayer',
            remappings=[("to_tf_global", "/tf"),
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static")],
            namespace=namespace
        ),
        # Node
        # (
        #     package='spice',
        #     executable='robot_state_manager_node.py',
        #     name='robot_state_manager_node',
        #     namespace=namespace
        # )
    ])
