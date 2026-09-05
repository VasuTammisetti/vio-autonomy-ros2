#!/usr/bin/env python3
"""
Latency harness -- the seed of the real-time dashboard.

Measures reaction latency: how old the most-recent LiDAR scan is at the
moment a velocity command is produced (scan stamp -> cmd_vel arrival),
on the /clock timeline. This is a proxy for perception->action latency;
Step B replaces it with true per-stage instrumentation.

Publishes each sample (ms) on /metrics/latency (std_msgs/Float64) and logs
p50/p95/p99 every 5 s. Real-time credibility = these numbers, not features.

use_sim_time is forced True so scan stamps and 'now' share one clock.
Subscribes to /cmd_vel as Twist (Nav2 Jazzy default). If /cmd_vel is
TwistStamped on your build, change the import/sub type accordingly.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def percentile(sorted_vals, pct):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * (pct / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = k - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


class LatencyHarness(Node):
    def __init__(self):
        super().__init__(
            "latency_harness",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.last_scan_t = None
        self.samples = []
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.pub = self.create_publisher(Float64, "/metrics/latency", 10)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            "latency harness up: measuring /scan -> /cmd_vel reaction latency (ms)"
        )

    def scan_cb(self, msg: LaserScan):
        self.last_scan_t = rclpy.time.Time.from_msg(msg.header.stamp)

    def cmd_cb(self, _msg: Twist):
        if self.last_scan_t is None:
            return
        dt_ms = (self.get_clock().now() - self.last_scan_t).nanoseconds / 1e6
        if 0.0 <= dt_ms < 5000.0:   # ignore clock-reset outliers
            self.samples.append(dt_ms)
            self.pub.publish(Float64(data=dt_ms))

    def report(self):
        if not self.samples:
            self.get_logger().info("latency: no cmd_vel yet -- set a goal in RViz")
            return
        window = sorted(self.samples[-500:])
        self.get_logger().info(
            "scan->cmd_vel latency ms  "
            f"p50={percentile(window, 50):.1f}  "
            f"p95={percentile(window, 95):.1f}  "
            f"p99={percentile(window, 99):.1f}  "
            f"n={len(self.samples)}"
        )


def main():
    rclpy.init()
    node = LatencyHarness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()