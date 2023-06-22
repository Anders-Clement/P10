import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

DEFAULT_MAP_NAME = "A4_new_65.yaml" # change to the name.yaml of the default map here

def generate_launch_description():

    map_name = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map',
            default_value = DEFAULT_MAP_NAME,
            description='Map name.yaml'
        ),

        Node(
            package='spice',
            executable='SwarmManager.py',
            name='SwarmManager'
        ),

        Node(
            package='spice',
            executable='work_cell_allocator',
            name='work_cell_allocator'
        ),

        Node(
            package='spice',
            executable='task_allocator',
            name='task_allocator'
        ),

        Node(
            package='spice',
            executable='work_cell_simulator',
            name='work_cell_simulator',
            parameters=[
                {'work_cell_rep_slope': 400.0}, # workcell repulsion for queue pos
                {'carrier_bot_rep_slope': 300.0}, # carrier bot repulsion
                {'wall_rep_slope': 400.0}, #static map obstacle repulsion
                {'queue_rep_slope': 270.0}, # other queue point repulsion
                {'plan_rep_slope': 275.0}, # robot plans repulsion
                {'work_cell_att_slope':  0.048}, # own workcell attraction
                {'queue_att_slope': 0.0}, #own lower queue point attraction
                {'min_move_dist': 10},
                {'q_max_vel': 0.5},
                {'map': map_name}
            ]
        ),

        Node(
            package='spice',
            executable='central_path_planner',
            name='central_path_planner',
            parameters=[
                {'priority_scheme': 0}, # 0 static priorites, 1 shortest dist hihgest priority, 2 planning failed gives highest priority 
                {'future_lookup': 0}, # number of higer priority plan steps to consider when planning 
                {'cost': 254} # cost of other higher priority robots' plans 
            ]
        ),
        Node(
            package='spice_mapf',
            executable='mapf_planner_node.py',
            name='mapf_planner_node'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('spice_nav'),
                    'launch/global_map_server.launch.py'
                )
            ),
            launch_arguments={'map': map_name}.items()
        )
    ])
