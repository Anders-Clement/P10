from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_viz'), 'rviz', 'set_job_poses.rviz']
    )
    return LaunchDescription([
        Node(
            package='spice',
            executable='set_robot_task_poses',
            name='set_robot_task_poses',
        ),
        Node(
            package='spice',
            executable='task_allocator',
            name='task_allocator',
            parameters=[
                {"use_rviz": True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],  
        ),

    ])