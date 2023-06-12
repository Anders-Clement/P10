from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_viz'), 'rviz', 'default_polybot_viz']
    )
    DeclareLaunchArgument(
        name = 'workcell',
        description='The workcell'
    )

    return LaunchDescription([
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', default_rviz_config_path],  
                    remappings=[
                        ('/goal_pose', [LaunchConfiguration('workcell'),'/goal_pose']),
                    ],
                ),
            ])