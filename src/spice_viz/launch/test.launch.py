import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory



def generate_launch_description():
    pb_nr = DeclareLaunchArgument(
        name = 'nr',
        default_value='4',
        description='The PolyBot\'s number'
    ),

    path = os.path.join(get_package_share_directory('spice_viz'), 'launch/rviz_polybot.launch.py')
    print(path)

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path
        )
    )
    #regular_node = Node( package='rviz2', executable='rviz2', name='rviz2',)

    return LaunchDescription([
        included_launch,
        #regular_node,
    ])