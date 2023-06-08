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

DEFAULT_MAP_NAME = 'A4.yaml' # change to the name.yaml of the default map here

def generate_launch_description():
    namespace = os.environ.get('ROBOT_NAMESPACE')
    if namespace is None:
        print('Failed to find ROBOT_NAMESPACE in environment, please add it!')
        return
    
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
      ]
    )
    
    return LaunchDescription([
        ns_bringup
    ])
