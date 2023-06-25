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

DEFAULT_MAP_NAME = 'A4_new.yaml' # change to the name.yaml of the default map here

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
            executable='polybot_agent.py',
            name='polybot_agent',
            parameters=[
                {"serial_port": '/dev/ttyACM0'}
            ]
        ),
         
      ]
    )
    
    return LaunchDescription([ 
        ns_bringup,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #      os.path.join(
        #          get_package_share_directory('spice_nav'),
        #          'launch/polybot_rsm_mapf.launch.py')
        #     )
        # )

    ])