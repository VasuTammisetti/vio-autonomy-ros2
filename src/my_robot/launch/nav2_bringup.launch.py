#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

PARAMS = "/home/vasu0/autonomy/src/my_robot/config/nav2_params.yaml"
TOOLS = "/home/vasu0/autonomy/tools"
RVIZ = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]

def generate_launch_description():
    servers = [
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
    ]
    manager = TimerAction(period=3.0, actions=[
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "bond_timeout": 0.0, "node_names": NODES}]),
    ])
    helpers = [
        ExecuteProcess(cmd=["python3", f"{TOOLS}/cmd_vel_bridge.py"], output="screen"),
        ExecuteProcess(cmd=["python3", f"{TOOLS}/latency_harness.py"], output="screen"),
        ExecuteProcess(cmd=["rviz2", "-d", RVIZ], output="log"),
    ]
    return LaunchDescription(servers + [manager] + helpers)
