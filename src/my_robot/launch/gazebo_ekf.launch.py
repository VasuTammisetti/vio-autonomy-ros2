import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'my_world.sdf')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = TimerAction(
        period=4.0,
        actions=[Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', '/robot_description', '-name', 'my_robot', '-z', '0.1'],
            output='screen',
        )],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen',
    )

    diff_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager-timeout', '120'],
        output='screen',
    )

    # The EKF: fuses wheel odom + IMU, publishes odom -> base_footprint
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo, robot_state_publisher, spawn, bridge,
        jsb_spawner, diff_spawner, ekf,
    ])
