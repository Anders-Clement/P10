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
        name = 'nr',
        description='The PolyBot\'s number'
    ),

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', ['/polybot', LaunchConfiguration('nr'), '/initialpose']),
                ('/goal_pose', ['polybot', LaunchConfiguration('nr'),'/goal_pose']),
                ('/tf', ['/polybot', LaunchConfiguration('nr'),'/tf']),
                ('/tf_static', ['/polybot', LaunchConfiguration('nr'),'/tf_static'])
            ],
        ),
    ])