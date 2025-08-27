import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    fake_robot_dir = get_package_share_directory("fake_robot")

    with open(os.path.join(fake_robot_dir, 'params', 'fake_robot.yaml')) as fake_robot_params:
        fake_robot_params = yaml.load(fake_robot_params, Loader=yaml.Loader)
        fake_robot_params = fake_robot_params['/**']['ros__parameters']['fake_robot']

    robot_name = LaunchConfiguration('robot_name')


    update_rate = fake_robot_params['update_rate']
    timeout = fake_robot_params['timeout']

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_0',
        description='Name of the robot'
    )

    fake_robot_node = Node(
        package='fake_robot',
        executable='fake_robot',
        name='fake_robot',
        namespace=robot_name,
        parameters=[
            {'update_rate': update_rate},
            {'timeout': timeout},
            {'robot_name': robot_name}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_name_arg)
    ld.add_action(fake_robot_node)

    return ld
