import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    real_robot_dir = get_package_share_directory("real_robot")

    with open(os.path.join(real_robot_dir, 'params', 'm2.yaml')) as real_robot_params:
        real_robot_params = yaml.load(real_robot_params, Loader=yaml.Loader)
        real_robot_params = real_robot_params['/**']['ros__parameters']['ISR_M2']

    robot_name = LaunchConfiguration("robot_name").perform(context)
    autostart = True

    port = real_robot_params['wheel_controller']['port']
    baudrate = real_robot_params['wheel_controller']['baudrate']

    odom_frame = robot_name + '/' + real_robot_params['odometry']['odom_frame']
    base_frame = robot_name + '/' + real_robot_params['odometry']['base_frame']

    ip_address = real_robot_params['laser']['ip_address']
    ip_port = real_robot_params['laser']['ip_port']
    laser_frame = robot_name + '/' + real_robot_params['laser']['laser_frame']
    scan_topic = '/' + robot_name + '/' + real_robot_params['laser']['scan_topic']
    laser_offset_x = real_robot_params['laser']['laser_offset_x']
    laser_offset_y = real_robot_params['laser']['laser_offset_y']
    laser_offset_z = real_robot_params['laser']['laser_offset_z']

    camera_frame = robot_name + '/' + real_robot_params['camera']['camera_frame']
    camera_name = robot_name + '/' + real_robot_params['camera']['camera_name']
    camera_offset_x = real_robot_params['camera']['camera_offset_x']
    camera_offset_y = real_robot_params['camera']['camera_offset_y']
    camera_offset_z = real_robot_params['camera']['camera_offset_z']

    isr_real_robot_node = Node(
        package='real_robot',
        executable='real_robot',
        name='real_robot',
        namespace=robot_name,
        parameters=[
            {'port': port},
            {'baudrate': baudrate},
            {'odom_frame': odom_frame},
            {'base_frame': base_frame},
        ],
        output='screen',
    )

    base_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_name,
        arguments=[
            laser_offset_x, laser_offset_y, laser_offset_z, # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            base_frame,
            laser_frame
        ],
        output='screen'
    )

    base_to_camera_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_name,
        arguments=[
            camera_offset_x,
            camera_offset_y,
            camera_offset_z,  # x, y, z
            "0",
            "0",
            "0",  # roll, pitch, yaw
            base_frame,
            camera_frame
        ],
        output="screen",
    )

    hokuyo_urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo_node',
        namespace=robot_name,
        parameters=[
            {'ip_address': ip_address},
            {'ip_port': ip_port},
            {'laser_frame_id': laser_frame},
        ],
        output='screen',
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera",
        namespace=robot_name,
        parameters=[
            {"pointcloud.enable": True},
            {"camera_name": camera_name},
        ],
        output="screen",
    )

    amcl_params = RewrittenYaml(
        source_file=os.path.join(real_robot_dir, 'params', 'amcl.yaml'),
        root_key=robot_name,
        param_rewrites={
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'scan_topic': scan_topic,
        },
        convert_types=True
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=robot_name,
        parameters=[
            amcl_params,
            {'autostart': autostart},
        ],
        output='screen'
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        emulate_tty=True,
        parameters=[
            {'autostart': autostart},
            {'node_names': [f'{robot_name}/amcl']},
        ],
        output='screen',
    )

    groups = []
    group = GroupAction(
        [
            isr_real_robot_node,
            base_to_laser_node,
            base_to_camera_node,
            hokuyo_urg_node,
            realsense_node,
            amcl_node,
            lifecycle_manager_node
        ]
    )
    groups.append(group)

    return groups

def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_0',
        description='Name of the robot'
    )

    opfunc = OpaqueFunction(function = launch_setup)

    ld = LaunchDescription()
    ld.add_action(robot_name_arg)
    ld.add_action(opfunc)

    return ld
