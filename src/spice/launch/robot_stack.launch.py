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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    namespace = os.environ.get('ROBOT_NAMESPACE')
    if namespace is None:
        print('Failed to find ROBOT_NAMESPACE in environment, please add it!')
        return
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('spice'),
                  'launch/robot_bringup.launch.py')
                )
        ),
        GroupAction(actions=[
          PushRosNamespace(namespace),
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('spice'),
                  'launch/tf_relay.launch.py')
                )
          ),
          Node
          (
            package='spice',
            executable='led_node.py',
            name='led_node'
          ),
          Node(
            package='spice',
            executable='robot_state_manager_node.py',
            name='robot_state_manager_node'
          ),
          Node(
            package='spice',
            executable='dynamic_obstacle_avoidance.py',
            name='dynamic_obstacle_avoidance_node'
          )
        ]),
        
      ])
