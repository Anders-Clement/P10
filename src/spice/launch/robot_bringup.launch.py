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
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

DEFAULT_MAP_NAME = 'canteen.yaml' # change to the name.yaml of the default map here

def generate_launch_description():
    map_name = LaunchConfiguration('map')
    map_path = LaunchConfiguration('map_path')

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'), 'maps', map_name]
    )

    return LaunchDescription([
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
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('linorobot2_bringup'),
                             'launch/namespace_bringup.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('spice_nav'),
                             'launch/navigation.launch.py')
            ),
            launch_arguments={
                'map_path': map_path
            }.items()
        ),
    ])
