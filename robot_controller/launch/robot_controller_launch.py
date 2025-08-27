import os
import yaml
import datetime

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_controller_dir = get_package_share_directory("robot_controller")

    with open(os.path.join(robot_controller_dir, 'params', 'robot_controller.yaml')) as robot_controller_params:
        robot_controller_params = yaml.load(robot_controller_params, Loader=yaml.Loader)
        robot_controller_params = robot_controller_params['/**']['ros__parameters']['robot_controller']

    robot_name = LaunchConfiguration('robot_name')

    # Get current datetime for log filename
    current_time = datetime.datetime.now()
    datetime_str = current_time.strftime("%Y_%m_%d_%H%M")

    # 소스 디렉토리의 logs 폴더 경로 (워크스페이스 기준이 아닌 실제 소스 디렉토리)
    logs_dir = os.path.join(os.path.join(os.path.dirname(os.path.dirname(robot_controller_dir)), '../../src'), 'logs')

    # logs 디렉토리가 없으면 생성
    if not os.path.exists(logs_dir):
        try:
            os.makedirs(logs_dir)
            print(f"Created logs directory at: {logs_dir}")
        except Exception as e:
            print(f"Error creating logs directory: {e}")

    log_filename = os.path.join(logs_dir, f"{datetime_str}.yaml")
    print(f"Log will be saved to: {log_filename}")

    lin_vel = robot_controller_params['lin_vel']
    ang_vel = robot_controller_params['ang_vel']
    min_lin_vel = robot_controller_params['min_lin_vel']
    max_lin_vel = robot_controller_params['max_lin_vel']
    lookahead_dist = robot_controller_params['lookahead_dist']

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_0',
        description='Name of the robot'
    )

    robot_controller_node = Node(
        package='robot_controller',
        executable='robot_controller',
        name='robot_controller',
        namespace=robot_name,
        parameters=[
            {'lin_vel': lin_vel},
            {'ang_vel': ang_vel},
            {'min_lin_vel': min_lin_vel},
            {'max_lin_vel': max_lin_vel},
            {'lookahead_dist': lookahead_dist},
            {'log_file_path': log_filename}
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_name_arg)
    ld.add_action(robot_controller_node)

    return ld
