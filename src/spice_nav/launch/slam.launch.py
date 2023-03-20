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
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'), 'config', 'slam.yaml']
    )

    robot_ns = os.getenv('ROBOT_NAMESPACE')
    if robot_ns is None:
        robot_ns = ""

    if robot_ns != "":
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata')
        ]
        slam_param_substitutions = {}

        slam_config = RewrittenYaml(
                source_file=slam_config_path,
                root_key=robot_ns,
                param_rewrites=slam_param_substitutions,
                convert_types=True)

        return LaunchDescription(
            [
                DeclareLaunchArgument(
                    name='sim', 
                    default_value='false',
                    description='Enable use_sime_time to true'
                ),
                GroupAction(actions=[
                    PushRosNamespace(robot_ns),
                    Node(
                        parameters=[
                            slam_config,
                            {'use_sim_time': LaunchConfiguration("sim")}
                        ],
                        package='slam_toolbox',
                        executable='async_slam_toolbox_node',
                        name='slam_toolbox',
                        output='screen',
                        remappings=remappings
                    )
                ])
            ])
    else:
        slam_config = slam_config_path
        remappings=[]

        return LaunchDescription([
            DeclareLaunchArgument(
                name='sim', 
                default_value='false',
                description='Enable use_sime_time to true'
            ),
            Node(
                parameters=[
                    slam_config,
                    {'use_sim_time': LaunchConfiguration("sim")}
                ],
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                remappings=remappings
            )
        ])