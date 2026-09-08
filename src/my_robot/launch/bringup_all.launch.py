#!/usr/bin/env python3
"""
bringup_all.launch.py -- ONE command to start the whole autonomy stack,
in the right order with timing so nodes come up cleanly.

    ros2 launch /home/vasu0/autonomy/src/my_robot/launch/bringup_all.launch.py

Optional args:
    use_predictions:=true    also start the prediction->costmap bridge (B.5).
                             Default false (predictions currently box the robot
                             in; we tune before enabling).
    use_mover:=true          also walk the person automatically. Default false
                             (we teleport the person manually to test safety).

Startup timeline:
    t=0s   Gazebo + EKF, robot_state_publisher, ros_gz_bridge
    t=5s   slam_toolbox localization
    t=10s  Nav2 servers + lifecycle manager (auto-activate)
    t=20s  cmd_vel bridge, person_publisher, world_model, safety_gate
    t=22s  (optional) prediction bridge / person mover
    t=3s   RViz
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
MAP = f"{HOME}/room_map_serial"
RVIZ = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
NAV2_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


def bash(cmd):
    """Run a shell command with the environment already sourced by the caller."""
    return ExecuteProcess(cmd=["bash", "-lc", cmd], output="screen")


def generate_launch_description():
    use_pred = LaunchConfiguration("use_predictions")
    use_mover = LaunchConfiguration("use_mover")

    # --- t=0: Gazebo + EKF (via existing launch) ---
    gazebo = bash(
        "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib && "
        f"ros2 launch {LAUNCH}/gazebo_ekf.launch.py"
    )

    # --- t=5: SLAM localization ---
    slam = TimerAction(period=5.0, actions=[bash(
        "ros2 launch slam_toolbox localization_launch.py use_sim_time:=true "
        f"map_file_name:={MAP} scan_topic:=/scan"
    )])

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
        bash(f"python3 {TOOLS}/person_publisher.py"),
        bash(f"python3 {TOOLS}/world_model_node.py"),
        bash(f"python3 {TOOLS}/safety_gate.py"),
        bash(f"python3 {TOOLS}/cmd_vel_bridge.py"),
    ])

    # --- t=22: optional prediction bridge + mover ---
    pred = TimerAction(period=22.0, actions=[
        bash(f"python3 {TOOLS}/prediction_costmap_bridge.py")
    ], condition=IfCondition(use_pred))
    mover = TimerAction(period=22.0, actions=[
        bash(f"python3 {TOOLS}/person_mover.py")
    ], condition=IfCondition(use_mover))

    # --- t=3: RViz ---
    rviz = TimerAction(period=3.0, actions=[
        ExecuteProcess(cmd=["rviz2", "-d", RVIZ], output="log")
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_predictions", default_value="false"),
        DeclareLaunchArgument("use_mover", default_value="false"),
        gazebo, slam, nav2_servers, nav2_manager, py_nodes, pred, mover, rviz,
    ])