from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import LifecycleNode

# from launch.actions import EmitEvent
# from launch_ros.events.lifecycle import ChangeState
# from launch_ros.event_handlers import OnStateTransition

# import launch
# import lifecycle_msgs.msg

MAP_NAME = 'C4.yaml' # Change name of map here

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')


        
    config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'config', 'global_map.yaml']
    )

    map_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'maps', MAP_NAME]
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'yaml_filename': map_path}

    configured_params = RewrittenYaml(
            source_file=config_path,
            param_rewrites=param_substitutions,
            convert_types=True)
    
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

        # EmitEvent(
        #     event = ChangeState(
        #     lifecycle_node_matcher = launch.events.process.matches_name('global_map_server'),
        #     transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        #     )
        # ),

        # EmitEvent(
        #     event = ChangeState(
        #     lifecycle_node_matcher = launch.events.process.matches_name('global_costmap'),
        #     transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        #     )
        # ),

        #     # Make the ZED node take the 'activate' transition
        # EmitEvent(
        #     event = ChangeState(
        #     lifecycle_node_matcher = launch.events.process.matches_name('global_map_server'),
        #     transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        #     )
        # ),

        # EmitEvent(
        #     event = ChangeState(
        #     lifecycle_node_matcher = launch.events.process.matches_name('global_costmap'),
        #     transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        #     )
        # )

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
    