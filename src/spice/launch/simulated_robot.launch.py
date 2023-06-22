        
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

    return LaunchDescription([ 
        Node(
            package='spice_mapf',
            executable='mapf_navigator_node_cpp',
            name='mapf_navigator_node',
            namespace=namespace,
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        Node
        (
            package='spice',
            executable='robot_state_manager_node.py',
            name='robot_state_manager_node',
            namespace=namespace
        ),
        Node(
            package='spice',
            executable='simulated_polybot_agent.py',
            name='simulated_polybot_agent',
            namespace=namespace,
            parameters=[
                {'initial_x': 7.0},
                {'initial_y': 4.0},
                {'initial_theta': 0.0}
            ]        
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