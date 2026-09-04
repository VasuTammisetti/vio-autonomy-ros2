import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')

    # xacro converts the .xacro file into plain URDF at launch time
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        # Publishes the fixed TF transforms from the URDF
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # GUI sliders to spin the wheel joints so you can watch TF move
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        # The 3D viewer
        Node(package='rviz2', executable='rviz2'),
    ])