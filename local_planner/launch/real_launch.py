import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    local_planner_dir = get_package_share_directory("local_planner")
    robot_controller_dir = get_package_share_directory("robot_controller")
    real_robot_dir = get_package_share_directory("real_robot")

    with open(os.path.join(local_planner_dir, 'params', 'local_planner.yaml')) as local_planner_params:
        local_planner_params = yaml.load(local_planner_params, Loader=yaml.Loader)
        local_planner_params = local_planner_params['/**']['ros__parameters']['local_planner']

    robot_name = LaunchConfiguration('robot_name')


    v_max = local_planner_params['v_max']
    w_max = local_planner_params['w_max']

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_0',
        description='Name of the robot'
    )

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

    real_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(real_robot_dir, 'launch', 'm2_launch.py')
        ),
        launch_arguments={
            'robot_name': robot_name,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(robot_name_arg)
    ld.add_action(local_planner_node)
    ld.add_action(robot_controller_node)
    ld.add_action(real_robot_node)

    return ld
