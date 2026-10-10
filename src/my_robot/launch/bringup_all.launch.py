#!/usr/bin/env python3
"""
bringup_all.launch.py -- ONE command to start the whole autonomy stack,
in the right order with timing so nodes come up cleanly.

    ros2 launch /home/vasu0/autonomy/src/my_robot/launch/bringup_all.launch.py

Optional args:
    use_predictions:=true|false  forecast -> Nav2 costmaps (local AND global). Default true.
    use_agents:=true|false       move the 3 people and the cart (tools/sim_agents.py) and feed
                                 their positions to the world model. Default true.
    use_mission:=true|false      start the autonomous patrol (tools/mission_runner.py). Default false,
                                 so you can check the stack first; then run it by hand or set true.

Startup timeline:
    t=0s   Gazebo + EKF, robot_state_publisher, ros_gz_bridge
    t=3s   RViz
    t=5s   slam_toolbox localization on the factory map
    t=10s  Nav2 servers, t=16s lifecycle manager (auto-activate)
    t=20s  sim_agents, world_model, safety_gate, cmd_vel bridge
    t=22s  prediction bridge
    t=35s  mission runner (if use_mission:=true)
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

HOME = "/home/vasu0/autonomy"
LAUNCH = f"{HOME}/src/my_robot/launch"
TOOLS = f"{HOME}/tools"
PARAMS = f"{HOME}/src/my_robot/config/nav2_params.yaml"
MAP = f"{HOME}/maps/factory_map_serial"   # factory hall 20 x 15 m (made 9 Oct)
# slam_toolbox localization reads the map path from its params file (map_file_name), not from a launch argument
SLAM_PARAMS = f"{HOME}/src/my_robot/config/slam_localization.yaml"
RVIZ = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
NAV2_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


def bash(cmd):
    """Run a shell command with the environment already sourced by the caller."""
    return ExecuteProcess(cmd=["bash", "-lc", cmd], output="screen")


def generate_launch_description():
    use_pred = LaunchConfiguration("use_predictions")
    use_agents = LaunchConfiguration("use_agents")
    use_mission = LaunchConfiguration("use_mission")
    use_vo = LaunchConfiguration("use_vo")

    # --- t=0: Gazebo + EKF (via existing launch) ---
    gazebo = bash(
        "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib && "
        f"ros2 launch {LAUNCH}/gazebo_ekf.launch.py"
    )

    # --- t=5: SLAM localization ---
    slam = TimerAction(period=5.0, actions=[bash(
        "ros2 launch slam_toolbox localization_launch.py use_sim_time:=true "
        f"slam_params_file:={SLAM_PARAMS}"
    )])

    # --- t=8: visual odometry (rtabmap rgbd_odometry) + relay into the EKF ---
    vo = TimerAction(period=8.0, actions=[
        bash("ros2 run rtabmap_odom rgbd_odometry --ros-args "
             f"--params-file {HOME}/src/my_robot/config/vo.yaml "
             "-r rgb/image:=/camera/image -r depth/image:=/camera/depth_image "
             "-r rgb/camera_info:=/camera/camera_info -r odom:=/vo/odom"),
        bash(f"python3 {TOOLS}/vo_relay.py"),
    ], condition=IfCondition(use_vo))

    # --- t=10: Nav2 servers + lifecycle manager ---
    nav2_servers = TimerAction(period=10.0, actions=[
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[PARAMS, {"use_sim_time": True}]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[PARAMS, {"use_sim_time": True}]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[PARAMS, {"use_sim_time": True}]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[PARAMS, {"use_sim_time": True}]),
    ])
    nav2_manager = TimerAction(period=16.0, actions=[
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "bond_timeout": 0.0, "node_names": NAV2_NODES}]),
    ])

    # --- t=20: python nodes (order: gate before bridge doesn't matter; both up) ---
    py_nodes = TimerAction(period=20.0, actions=[
        bash(f"python3 {TOOLS}/world_model_node.py"),
        bash(f"python3 {TOOLS}/safety_gate.py"),
        bash(f"python3 {TOOLS}/cmd_vel_bridge.py"),
    ])

    # sim_agents replaces person_publisher + person_mover: it moves all agents AND publishes /fused_objects.
    # Never run it together with person_publisher.py (that would double the detections).
    agents = TimerAction(period=20.0, actions=[
        bash(f"python3 {TOOLS}/sim_agents.py")
    ], condition=IfCondition(use_agents))

    # --- t=22: prediction bridge ---
    pred = TimerAction(period=22.0, actions=[
        bash(f"python3 {TOOLS}/prediction_costmap_bridge.py")
    ], condition=IfCondition(use_pred))
    # --- t=35: autonomous patrol ---
    mission = TimerAction(period=35.0, actions=[
        bash(f"python3 {TOOLS}/mission_runner.py")
    ], condition=IfCondition(use_mission))

    # --- t=3: RViz ---
    rviz = TimerAction(period=3.0, actions=[
        ExecuteProcess(cmd=["rviz2", "-d", RVIZ], output="log")
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_predictions", default_value="true"),
        DeclareLaunchArgument("use_agents", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("use_vo", default_value="true"),
        gazebo, slam, vo, nav2_servers, nav2_manager, py_nodes, agents, pred, mission, rviz,
    ])