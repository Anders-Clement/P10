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
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

MAP_NAME='C4' #change to the name of your own map here

def generate_launch_description():
    #depth_sensor = os.getenv('LINOROBOT2_DEPTH_SENSOR', '')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('spice'), 'launch', 'nav_bringup.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config', 'navigation.yaml']
    )

    robot_ns = os.environ.get('ROBOT_NAMESPACE')
    if robot_ns is None:
        robot_ns = ""

    if robot_ns != "":
        use_namespace = 'true'
    else:
        use_namespace = 'false'

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'namespace' : robot_ns,
                'use_namespace' : use_namespace,
                'use_composition' : 'True',
                'params_file': nav2_config_path,
                'autostart': 'True'
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])