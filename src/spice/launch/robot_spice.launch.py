from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        #Node(
           # package='spice',
            #executable='robot_tf_relay',
          #  name='tf_relay',
       # ),
        Node(
            package='spice',
            executable='robot_tf_pose',
            name='robot_pose_relayer',
            remappings=[("to_tf_global", "/tf"),("/tf", "tf")]
        )

    ])