#!/usr/bin/env python3
"""
Minimal Nav2 bring-up for my_robot.
Assumes Gazebo+EKF (gazebo_ekf.launch.py) and slam_toolbox localization
are ALREADY running in other terminals (they publish odom->base_footprint
and map->odom, plus /scan and /map).

Starts: controller_server, planner_server, behavior_server, bt_navigator,
lifecycle_manager, the cmd_vel bridge, the latency harness, and RViz.

Run by file path (no colcon rebuild needed):
    ros2 launch ~/autonomy/src/my_robot/launch/nav2.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HOME = os.path.expanduser("~")
PARAMS = os.path.join(HOME, "autonomy/src/my_robot/config/nav2_params.yaml")
TOOLS = os.path.join(HOME, "autonomy/tools")
RVIZ_CFG = "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"

LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description():
    nav2_nodes = [
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[PARAMS]),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[PARAMS]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[PARAMS]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[PARAMS]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True,
                          "autostart": True,
                          "node_names": LIFECYCLE_NODES}]),
    ]

    helpers = [
        # Twist /cmd_vel  ->  TwistStamped /diff_drive_controller/cmd_vel
        ExecuteProcess(cmd=["python3", os.path.join(TOOLS, "cmd_vel_bridge.py")],
                       output="screen"),
        # scan->cmd_vel latency, published on /metrics/latency
        ExecuteProcess(cmd=["python3", os.path.join(TOOLS, "latency_harness.py")],
                       output="screen"),
        # RViz with the Nav2 default view (has the "Nav2 Goal" tool + costmaps)
        ExecuteProcess(cmd=["rviz2", "-d", RVIZ_CFG], output="log"),
    ]

    return LaunchDescription(nav2_nodes + helpers)