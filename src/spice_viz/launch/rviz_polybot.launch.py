from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# Could not find a way to get the cmd arg and use it as string. :(
# So made a fix to make it work by just adding a new
# condition check per robot ID. 
# This should be changed to the scalable solution if a method is found.
def generate_launch_description():
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('spice_viz'), 'rviz', 'default_polybot_viz']
    )
    DeclareLaunchArgument(
        name = 'nr',
        description='The PolyBot\'s number'
    ),
    nr = LaunchConfiguration('nr')


    return LaunchDescription([
        #included_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot01/initialpose'),
                ('/goal_pose', 'polybot01/goal_pose'),
                ('/tf', '/polybot01/tf'),
                ('/tf_static', '/polybot01/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 1'
                ])
            ),
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot02/initialpose'),
                ('/goal_pose', 'polybot02/goal_pose'),
                ('/tf', '/polybot02/tf'),
                ('/tf_static', '/polybot02/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 2'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot03/initialpose'),
                ('/goal_pose', 'polybot03/goal_pose'),
                ('/tf', '/polybot03/tf'),
                ('/tf_static', '/polybot03/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 3'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot04/initialpose'),
                ('/goal_pose', 'polybot04/goal_pose'),
                ('/tf', '/polybot04/tf'),
                ('/tf_static', '/polybot04/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 4'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot05/initialpose'),
                ('/goal_pose', 'polybot05/goal_pose'),
                ('/tf', '/polybot05/tf'),
                ('/tf_static', '/polybot05/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 5'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot06/initialpose'),
                ('/goal_pose', 'polybot06/goal_pose'),
                ('/tf', '/polybot06/tf'),
                ('/tf_static', '/polybot06/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 6'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot07/initialpose'),
                ('/goal_pose', 'polybot07/goal_pose'),
                ('/tf', '/polybot07/tf'),
                ('/tf_static', '/polybot07/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 7'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot08/initialpose'),
                ('/goal_pose', 'polybot08/goal_pose'),
                ('/tf', '/polybot08/tf'),
                ('/tf_static', '/polybot08/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 8'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot09/initialpose'),
                ('/goal_pose', 'polybot09/goal_pose'),
                ('/tf', '/polybot09/tf'),
                ('/tf_static', '/polybot09/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 9'
                ])
            ),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config_path],  
            remappings=[
                ('/initialpose', '/polybot10/initialpose'),
                ('/goal_pose', 'polybot10/goal_pose'),
                ('/tf', '/polybot10/tf'),
                ('/tf_static', '/polybot10/tf_static')
            ],
            condition=IfCondition(
                PythonExpression([
                    nr,
                    ' == 10'
                ])
            ),
        ),
    ])