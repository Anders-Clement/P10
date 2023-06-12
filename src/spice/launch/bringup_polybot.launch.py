import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

DEFAULT_MAP_NAME = 'low_res/A4_nav.yaml' # change to the name.yaml of the default map here

def generate_launch_description():
    namespace = os.environ.get('ROBOT_NAMESPACE')
    if namespace is None:
        print('Failed to find ROBOT_NAMESPACE in environment, please add it!')
        return
    
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

    # DeclareLaunchArgument(
    #     name = 'nr',
    #     description='The PolyBot\'s number'
    # ),

    use_namespace = 'true'
    #namespace = ['polybot', LaunchConfiguration("nr")]

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
    
    rplidar2_launch_path = PathJoinSubstitution(
        [FindPackageShare('rplidar_ros2'), 'launch', 'rplidar_a3_launch.py']
    )
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )    

    ns_bringup=GroupAction(actions=[
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar2_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        Node(
            package='spice',
            executable='robot_tf_pose',
            name='robot_pose_relayer',
            remappings=[("to_tf_global", "/tf"),
                        ("/tf", "tf"),
                        ("/tf_static", "tf_static")],
        ),
        Node(
            package='spice',
            executable='polybot_agent.py',
            name='polybot_agent',
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
      ]
    )
    
    return LaunchDescription([
        ns_bringup
    ])
