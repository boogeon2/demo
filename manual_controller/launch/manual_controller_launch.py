import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    manual_controller_dir = get_package_share_directory("manual_controller")

    with open(os.path.join(manual_controller_dir, 'params', 'manual_controller.yaml')) as manual_controller_params:
        manual_controller_params = yaml.load(manual_controller_params, Loader=yaml.Loader)
        manual_controller_params = manual_controller_params['/**']['ros__parameters']['manual_controller']

    robot_name = LaunchConfiguration("robot_name")

    delta_v = manual_controller_params['keyboard']['delta_v']
    delta_w = manual_controller_params['keyboard']['delta_w']

    joy_axis_linear = manual_controller_params['joystick']['joy_axis_linear']
    joy_axis_angular = manual_controller_params['joystick']['joy_axis_angular']
    joy_button_move = manual_controller_params['joystick']['joy_button_move']
    joy_button_help = manual_controller_params['joystick']['joy_button_help']
    joy_button_toggle = manual_controller_params['joystick']['joy_button_toggle']
    joy_scale_linear = manual_controller_params['joystick']['joy_scale_linear']
    joy_scale_angular = manual_controller_params['joystick']['joy_scale_angular']
    joy_deadzone = manual_controller_params['joystick']['joy_deadzone']
    joy_device_index = manual_controller_params['joystick']['joy_device_index']

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_0',
        description='Name of the robot',
    )

    manual_controller_node = Node(
        package='manual_controller',
        executable='manual_controller',
        name='manual_controller',
        namespace=robot_name,
        parameters=[
            {'delta_v': delta_v},
            {'delta_w': delta_w},
            {'joy_axis_linear': joy_axis_linear},
            {'joy_axis_angular': joy_axis_angular},
            {'joy_button_move': joy_button_move},
            {'joy_button_help': joy_button_help},
            {'joy_button_toggle': joy_button_toggle},
            {'joy_scale_linear': joy_scale_linear},
            {'joy_scale_angular': joy_scale_angular},
            {'joy_deadzone': joy_deadzone},
            {'joy_device_index': joy_device_index},
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(robot_name_arg)
    ld.add_action(manual_controller_node)

    return ld
