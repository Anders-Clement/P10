        
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_MAP_NAME = 'low_res/A4_new_nav.yaml' # change to the name.yaml of the default map here

def generate_launch_description():

    DeclareLaunchArgument(
        name = 'nr',
        description='The PolyBot\'s number'
    ),

    namespace = ['polybot', LaunchConfiguration("nr")]
    map_name = LaunchConfiguration('map')
    map_path = LaunchConfiguration('map_path')

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'), 'maps', map_name]
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'launch', 'nav_bringup.launch.py']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_nav'),
         'config', 'navigation.yaml']
    )

    use_namespace = 'true'

    return LaunchDescription([ 
        Node(
            package='spice_mapf',
            executable='mapf_navigator_node.py',
            name='mapf_navigator_node',
            namespace=namespace,
            # remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        Node
        (
            package='spice',
            executable='robot_state_manager_node.py',
            name='robot_state_manager_node',
            namespace=namespace
        ),
        DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sime_time to true'
        ),

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
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map_path': map_path,
                'use_sim_time': LaunchConfiguration("sim"),
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_composition': 'True',
                'params_file': nav2_config_path,
                'namespace' : namespace,
                'autostart' : 'True',
                'run_nav_stack' : 'False'
            }.items()
        ),
        Node(
            package='spice',
            executable='robot_tf_pose',
            name='robot_pose_relayer',
            remappings=[("to_tf_global", "/tf"),
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static")],
            namespace=namespace
        )
    ])