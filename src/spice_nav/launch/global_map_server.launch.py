from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LifecycleNode
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


MAP_NAME = 'canteen.yaml' # Change name of map here

def generate_launch_description():


    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')


        
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
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='log level'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )


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
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
    
        map_server_node,
        costmap_node,

    ## Option A
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

    ## Option B if the other crashed is to have this (and maybe change lifecyclenodes to nodes)
        ## lifecycleManager that activates the noted and then dies insead
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', log_level],
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'autostart': autostart},
        #         {'node_names': [costmap_node, map_server_node]}]
        # )
    ])
    