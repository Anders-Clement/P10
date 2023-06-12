        
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    DeclareLaunchArgument(
        name = 'nr',
        description='The PolyBot\'s number'
    ),

    namespace = ['polybot', LaunchConfiguration("nr")]

    return LaunchDescription([ 
        Node(
            package='spice_mapf',
            executable='mapf_navigator_node.py',
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
        )
    ])