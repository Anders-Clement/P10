import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')


        
    config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'config', 'global_map.yaml']
    )
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        name='use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='log level')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Enable use_sime_time to true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')


    return LaunchDescription([
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='global_map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[config_path],
            arguments=['--ros-args', '--log-level', log_level]
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            parameters=[config_path],
            arguments=['--ros-args', '--log-level', log_level]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_global_map',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['global_map_server','costmap/costmap']}]),
    ])
    

