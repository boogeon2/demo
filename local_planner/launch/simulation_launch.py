import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    local_planner_dir = get_package_share_directory("local_planner")
    robot_controller_dir = get_package_share_directory("robot_controller")
    fake_robot_dir = get_package_share_directory("fake_robot")

    with open(os.path.join(local_planner_dir, 'params', 'local_planner.yaml')) as local_planner_params:
        local_planner_params = yaml.load(local_planner_params, Loader=yaml.Loader)
        local_planner_params = local_planner_params['/**']['ros__parameters']['local_planner']

    robot_num = LaunchConfiguration('robot_num').perform(context)


    v_max = local_planner_params['v_max']
    w_max = local_planner_params['w_max']

    groups = []
    for i in range(0, int(robot_num)):
        robot_name = 'robot_' + str(i)

        local_planner_node = Node(
            package='local_planner',
            executable='local_planner',
            name='local_planner',
            namespace=robot_name,
            parameters=[
                {'v_max': v_max},
                {'w_max': w_max},
            ],
            output='screen'
        )

        robot_controller_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_controller_dir, 'launch', 'robot_controller_launch.py')
            ),
            launch_arguments={
                'robot_name': robot_name,
            }.items()
        )

        fake_robot_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fake_robot_dir, 'launch', 'fake_robot_launch.py')
            ),
            launch_arguments={
                'robot_name': robot_name,
            }.items()
        )

        group = GroupAction(
            [
                local_planner_node,
                robot_controller_node,
                fake_robot_node
            ]
        )
        groups.append(group)

    return groups

def generate_launch_description():
    robot_num_arg = DeclareLaunchArgument(
        'robot_num',
        default_value='1',
        description='Number of robots'
    )

    opfunc = OpaqueFunction(function = launch_setup)

    ld = LaunchDescription()
    ld.add_action(robot_num_arg)
    ld.add_action(opfunc)

    return ld
