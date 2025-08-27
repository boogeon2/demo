import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    global_planner_dir = get_package_share_directory('global_planner')

    with open(os.path.join(global_planner_dir, 'params', 'global_planner.yaml')) as global_planner_params:
        global_planner_params = yaml.load(global_planner_params, Loader=yaml.Loader)
        global_planner_params = global_planner_params['/**']['ros__parameters']['global_planner']

    robot_num = LaunchConfiguration('robot_num')

    autostart = True

    server_mode = global_planner_params['server_mode']

    cfg_file_name = global_planner_dir + global_planner_params['param_files']['cfg_file_name']
    map_file_name = global_planner_dir + global_planner_params['param_files']['map_file_name']
    graph_info_file = global_planner_dir + global_planner_params['param_files']['graph_info_file']
    guide_path_file = global_planner_dir + global_planner_params['param_files']['guide_path_file']

    map_name = global_planner_params['map']['map_name']
    env_height = global_planner_params['map']['env_height']
    map_resolution = global_planner_params['map']['map_resolution']

    task_seed = global_planner_params['task_assigner']['task_seed']
    max_task_count = global_planner_params['task_assigner']['max_task_count']
    task_per_period = global_planner_params['task_assigner']['task_per_period']
    task_period = global_planner_params['task_assigner']['task_period']
    capacity = global_planner_params['task_assigner']['capacity']

    agent_size = global_planner_params[server_mode]['agent_size']

    robot_num_arg = DeclareLaunchArgument(
        'robot_num',
        default_value='1',
        description='Number of robots'
    )

    map_server_param = RewrittenYaml(
        source_file=os.path.join(
            global_planner_dir, 'maps', map_name, 'map_server_params.yaml'),
        root_key='',
        param_rewrites={
            'yaml_filename': os.path.join(
                global_planner_dir, 'maps', map_name, 'map.yaml')
        },
        convert_types=True
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[map_server_param,
                    {'autostart': autostart}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        emulate_tty=True,
        parameters=[{'autostart': autostart},
                    {'node_names': ['map_server']}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            global_planner_dir,
            'rviz',
            map_name+'.rviz'
        )],
        parameters=[{'autostart': autostart}],
    )

    global_planner_node = Node(
        package='global_planner',
        executable='global_planner',
        name='global_planner',
        output='screen',
        prefix=['gdbserver localhost:3000'],
        parameters=[
            {'robot_num': robot_num},
            {'server_mode': server_mode},
            {'cfg_file_name': cfg_file_name},
            {'map_file_name': map_file_name},
            {'graph_info_file': graph_info_file},
            {'guide_path_file': guide_path_file},
            {'env_height': env_height},
            {'map_resolution': map_resolution},
            {'task_seed': task_seed},
            {'max_task_count': max_task_count},
            {'task_per_period': task_per_period},
            {'task_period': task_period},
            {'capacity': capacity},
            {'agent_size': agent_size}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(robot_num_arg)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_node)
    ld.add_action(rviz_node)
    ld.add_action(global_planner_node)

    return ld
