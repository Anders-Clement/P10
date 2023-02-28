import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition

MAP_NAME='C4' #change to the name of your own map here


def generate_launch_description():
    
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('spice'), 'launch', 'nav2_bringup_v2.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_navigation.rviz']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('spice'), 'config', 'navigation.yaml']
    )


    namespace = os.environ.get('ROBOT_NAMESPACE')
    if namespace is None:
        print('Failed to find ROBOT_NAMESPACE in environment, please add it!')
        return
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('linorobot2_bringup'),
                  'launch/namespace_bringup.launch.py')
                )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('linorobot2_navigation'),
                  'launch/navigation.launch.py')
                )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path), ###CHANGE TO NAV2 bringUP NO MAP_SERVER
            launch_arguments={
                'use_sim_time': LaunchConfiguration("sim"),
                'namespace' : namespace,
                'use_namespace' : 'true',
                'use_composition' : 'False',
                'params_file': nav2_config_path
            }.items()
        ),

        GroupAction(actions=[
          PushRosNamespace(namespace),
          IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('spice'),
                  'launch/tf_relay.launch.py')
                )
          ),
          Node
          (
            package='spice',
            executable='led_node.py',
            name='led_node'
          ),
          Node(
            package='spice',
            executable='robot_state_manager_node.py',
            name='robot_state_manager_node'
          ),
           Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
        ]),
        
      ])
