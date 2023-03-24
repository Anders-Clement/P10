from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LifecycleNode
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


MAP_NAME = 'C4.yaml' # Change name of map here

def generate_launch_description():


    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')


        
    config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'config', 'global_map.yaml']
    )

    map_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'maps', MAP_NAME]
    )

    ## Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'yaml_filename': map_path}

    configured_params = RewrittenYaml(
            source_file=config_path,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    ## Launch Arguements
    declare_use_respawn_cmd = DeclareLaunchArgument(
        name='use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='log level')

    ## Nodes
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='global_map_server',
        namespace="",
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
    )

    costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace="",
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        declare_use_respawn_cmd,
        declare_log_level_cmd, 
    
        map_server_node,
        costmap_node,


        ## Global map server configure and activated
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/global_map_server/change_state ",
                "lifecycle_msgs/srv/ChangeState ",
                '"{transition: {id: 1}}"',
                '&&',
                FindExecutable(name='ros2'),
                " service call ",
                "/global_map_server/change_state ",
                "lifecycle_msgs/srv/ChangeState ",
                '"{transition: {id: 3}}"',
            ]],
            shell=True
        ),  


        ## Global costmap configure and activated
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/costmap/costmap/change_state ",
                "lifecycle_msgs/srv/ChangeState ",
                '"{transition: {id: 1}}"',
                '&&',
                FindExecutable(name='ros2'),
                " service call ",
                "/costmap/costmap/change_state ",
                "lifecycle_msgs/srv/ChangeState ",
                '"{transition: {id: 3}}"',
            ]],
            shell=True
        ),  
    ])
    