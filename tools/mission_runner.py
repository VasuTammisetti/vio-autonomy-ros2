#!/usr/bin/env python3
"""
mission_runner -- makes the robot fully autonomous: it patrols the stations from factory_layout.py
(assembly -> loading bay 1 -> charging -> QA cell -> ...) through Nav2, forever, with no human input.

  - sets the initial pose (robot spawns at the hall centre facing east), waits for Nav2
  - sends one NavigateToPose goal per station. If it fails (e.g. someone was standing in the way),
    it clears the costmaps, waits RETRY_WAIT s and tries again, up to MAX_TRIES times; after that, or after
    LEG_TIMEOUT, it goes on to the next station (logged, never waits for a human)
  - logs every leg to ~/autonomy/logs/mission_<time>.csv: result, sim time, Nav2 recoveries,
    closest distance to any tracked person/cart, and time spent in each safety-gate state.
    Those numbers go on the slides.

Run after the stack is up (bringup_all.launch.py):
    python3 ~/autonomy/tools/mission_runner.py            # loop forever (Ctrl+C to stop)
    python3 ~/autonomy/tools/mission_runner.py --laps 2   # stop after 2 laps
"""
import argparse
import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from factory_layout import STATIONS

LEG_TIMEOUT = 240.0   # s of sim time per station before giving up on it
MAX_TRIES = 3         # attempts per station
RETRY_WAIT = 3.0      # s of sim time between attempts
DWELL = 3.0           # s of sim time at each station
LOG_DIR = os.path.expanduser("~/autonomy/logs")


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def sim_now(nav):
    return nav.get_clock().now().nanoseconds * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--laps", type=int, default=0, help="number of laps, 0 = forever")
    ap.add_argument("--no-initial-pose", action="store_true", help="do not publish the start pose")
    args = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator("mission_runner")
    nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    state = {"gate": "AUTONOMOUS", "tracks": []}
    nav.create_subscription(String, "/safety/state", lambda m: state.__setitem__("gate", m.data), 10)
    nav.create_subscription(MarkerArray, "/world_model/tracks", lambda m: state.__setitem__(
        "tracks", [(k.pose.position.x, k.pose.position.y) for k in m.markers if k.ns == "track"]), 10)

    if not args.no_initial_pose:
        nav.setInitialPose(make_pose(nav, 0.0, 0.0, 0.0))
    nav.waitUntilNav2Active(localizer="robot_localization")  # slam_toolbox localizes; skip the AMCL check
    nav.get_logger().info("Nav2 active, starting patrol: " + " -> ".join(s[0] for s in STATIONS))

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime("mission_%Y%m%d_%H%M%S.csv"))
    f = open(path, "w", newline="")
    log = csv.writer(f)
    log.writerow(["lap", "station", "result", "attempts", "sim_time_s", "recoveries", "min_dist_to_agent_m",
                  "t_degraded_s", "t_safe_stop_s", "t_hold_s"])

    lap, i = 1, 0
    try:
        while rclpy.ok():
            name, x, y, yaw = STATIONS[i]
            t0 = sim_now(nav)
            last = t0
            gate_time = {"DEGRADED": 0.0, "SAFE_STOP": 0.0, "HOLD": 0.0}
            min_d, recov = float("inf"), 0
            nav.get_logger().info("lap %d: going to %s (%.1f, %.1f)" % (lap, name, x, y))
            result, attempt, recov_total = "", 0, 0
            while attempt < MAX_TRIES and rclpy.ok():
                attempt += 1
                if attempt > 1:
                    nav.get_logger().warn("%s: %s, clearing costmaps and retrying (%d/%d)" % (name, result, attempt, MAX_TRIES))
                    nav.clearAllCostmaps()
                    t_wait = sim_now(nav) + RETRY_WAIT
                    while sim_now(nav) < t_wait and rclpy.ok():
                        rclpy.spin_once(nav, timeout_sec=0.1)
                    last = sim_now(nav)
                recov = 0
                if not nav.goToPose(make_pose(nav, x, y, yaw)):
                    result = "REJECTED"
                    continue
                timed_out = False
                while not nav.isTaskComplete():
                    now = sim_now(nav)
                    if state["gate"] in gate_time:
                        gate_time[state["gate"]] += max(0.0, now - last)
                    last = now
                    fb = nav.getFeedback()
                    if fb:
                        recov = fb.number_of_recoveries
                        rx, ry = fb.current_pose.pose.position.x, fb.current_pose.pose.position.y
                        for (tx, ty) in state["tracks"]:
                            min_d = min(min_d, math.hypot(tx - rx, ty - ry))
                    if now - t0 > LEG_TIMEOUT:
                        nav.get_logger().warn("%s: timeout, skipping to the next station" % name)
                        nav.cancelTask()
                        timed_out = True
                        break
                recov_total += recov
                r = nav.getResult()
                result = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "TIMEOUT",
                          TaskResult.FAILED: "FAILED"}.get(r, str(r))
                if result == "SUCCEEDED" or timed_out:
                    break
            recov = recov_total
            dt = sim_now(nav) - t0
            log.writerow([lap, name, result, attempt, "%.1f" % dt, recov, "%.2f" % min_d if min_d < 1e9 else "",
                          "%.1f" % gate_time["DEGRADED"], "%.1f" % gate_time["SAFE_STOP"], "%.1f" % gate_time["HOLD"]])
            f.flush()
            nav.get_logger().info("%s: %s in %.0f s (attempts %d), recoveries %d, closest agent %.2f m"
                                  % (name, result, dt, attempt, recov, min_d if min_d < 1e9 else -1))
            t_end = sim_now(nav) + (DWELL if result == "SUCCEEDED" else 1.0)
            while sim_now(nav) < t_end and rclpy.ok():
                rclpy.spin_once(nav, timeout_sec=0.1)
            i = (i + 1) % len(STATIONS)
            if i == 0:
                if args.laps and lap >= args.laps:
                    break
                lap += 1
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        f.close()
        print("log written to", path)
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()