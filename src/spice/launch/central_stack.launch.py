import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

DEFAULT_MAP_NAME = "C4.yaml" # change to the name.yaml of the default map here

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

        # Node(
        #     package='spice',
        #     executable='work_cell_simulator',
        #     name='work_cell_simulator',
        #     parameters=[{'work_cell_rep_slope': 0.05}, # workcell repulsion for queue pos
        #                 {'carrier_bot_rep_slope': 0.1}, # carrier bot repulsion
        #                 {'wall_rep_slope': 0.1}, #static map obstacle repulsion
        #                 {'queue_rep_slope': 0.1}, # other queue point repulsion
        #                 {'plan_rep_slope': 0.1}, # robot plans repulsion
        #                 {'work_cell_att_slope':  0.05}, # own workcell attraction
        #                 {'queue_att_slope': 0.0}, #own lower queue point attraction
        #                 {'min_move_dist': 5},
        #                 {'q_max_vel': 0.33},
        #                 {'map': map_name}

        #     ]
        # ),

        Node(
            package='spice',
            executable='central_path_planner',
            name='central_path_planner',
            parameters=[{'priority_scheme': 1}, # 0 static priorites, 1 shortest dist hihgest priority, 2 planning failed gives highest priority 
                        {'future_lookup': 0}, # number of higer priority plan steps to consider when planning 
                        {'cost': 128} # number of higer priority plan steps to consider when planning 
            ]
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
