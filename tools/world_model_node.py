#!/usr/bin/env python3
"""World Model node -- tracking + forecasting from /fused_objects."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

GATE_RADIUS = 1.0
MAX_MISSED = 5
FORECAST_STEPS = 10
FORECAST_DT = 0.5
PROCESS_NOISE = 0.5
MEAS_NOISE = 0.1
MIN_SPEED_TO_DRAW = 0.05


class Track:
    _next_id = 0

    def __init__(self, x, y, name, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        self.name = name
        self.X = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.5, 0.5, 2.0, 2.0])
        self.missed = 0
        self.last_stamp = stamp

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        G = np.array([0.5 * dt * dt, 0.5 * dt * dt, dt, dt])
        Q = np.outer(G, G) * PROCESS_NOISE
        self.X = F @ self.X
        self.P = F @ self.P @ F.T + Q

    def update(self, zx, zy):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.diag([MEAS_NOISE, MEAS_NOISE])
        z = np.array([zx, zy])
        y = z - H @ self.X
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X = self.X + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.missed = 0

    def speed(self):
        return math.hypot(self.X[2], self.X[3])

    def forecast(self, steps, dt):
        Xf = self.X.copy(); Pf = self.P.copy()
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        G = np.array([0.5 * dt * dt, 0.5 * dt * dt, dt, dt])
        Q = np.outer(G, G) * PROCESS_NOISE
        out = []
        for _ in range(steps):
            Xf = F @ Xf; Pf = F @ Pf @ F.T + Q
            out.append((Xf[0], Xf[1], Pf[0, 0], Pf[1, 1], Pf[0, 1]))
        return out


class WorldModel(Node):
    def __init__(self):
        super().__init__("world_model",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tracks = []
        self.create_subscription(MarkerArray, "/fused_objects", self.on_objects, 10)
        self.track_pub = self.create_publisher(MarkerArray, "/world_model/tracks", 10)
        self.pred_pub = self.create_publisher(MarkerArray, "/world_model/predictions", 10)
        self.last_t = None
        self.get_logger().info("world_model up: /fused_objects -> tracks + predictions")

    def on_objects(self, msg):
        dets = []
        for m in msg.markers:
            if m.type == Marker.TEXT_VIEW_FACING:
                continue
            dets.append((m.pose.position.x, m.pose.position.y, m.ns))
        now = self.get_clock().now()
        dt = 0.1 if self.last_t is None else max(1e-3, (now - self.last_t).nanoseconds / 1e9)
        self.last_t = now
        for t in self.tracks:
            t.predict(dt)
        used = set()
        for (dx, dy, name) in dets:
            best, best_d = None, GATE_RADIUS
            for t in self.tracks:
                if t.id in used:
                    continue
                d = math.hypot(t.X[0] - dx, t.X[1] - dy)
                if d < best_d:
                    best, best_d = t, d
            if best is not None:
                best.update(dx, dy); used.add(best.id)
            else:
                self.tracks.append(Track(dx, dy, name, now))
        for t in self.tracks:
            if t.id not in used:
                t.missed += 1
        self.tracks = [t for t in self.tracks if t.missed <= MAX_MISSED]
        self.publish_tracks()
        self.publish_predictions()

    def publish_tracks(self):
        arr = MarkerArray(); mid = 0
        for t in self.tracks:
            c = Marker()
            c.header.frame_id = "map"; c.ns = "track"; c.id = mid; mid += 1
            c.type = Marker.CUBE; c.action = Marker.ADD
            c.pose.position.x = float(t.X[0]); c.pose.position.y = float(t.X[1]); c.pose.position.z = 0.2
            c.pose.orientation.w = 1.0
            c.scale.x = c.scale.y = c.scale.z = 0.25
            c.color.g = 1.0; c.color.a = 0.9
            c.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            arr.markers.append(c)
            if t.speed() > MIN_SPEED_TO_DRAW:
                a = Marker()
                a.header.frame_id = "map"; a.ns = "vel"; a.id = mid; mid += 1
                a.type = Marker.ARROW; a.action = Marker.ADD
                a.scale.x = 0.05; a.scale.y = 0.1; a.scale.z = 0.1
                a.color.r = 1.0; a.color.g = 1.0; a.color.a = 1.0
                a.points = [Point(x=float(t.X[0]), y=float(t.X[1]), z=0.2),
                            Point(x=float(t.X[0] + t.X[2]), y=float(t.X[1] + t.X[3]), z=0.2)]
                a.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                arr.markers.append(a)
        if arr.markers:
            self.track_pub.publish(arr)

    def publish_predictions(self):
        arr = MarkerArray(); mid = 0
        for t in self.tracks:
            if t.speed() < MIN_SPEED_TO_DRAW:
                continue
            fc = t.forecast(FORECAST_STEPS, FORECAST_DT)
            for i, (fx, fy, vxx, vyy, vxy) in enumerate(fc):
                e = Marker()
                e.header.frame_id = "map"; e.ns = "pred_cov"; e.id = mid; mid += 1
                e.type = Marker.CYLINDER; e.action = Marker.ADD
                e.pose.position.x = float(fx); e.pose.position.y = float(fy); e.pose.position.z = 0.05
                e.pose.orientation.w = 1.0
                e.scale.x = float(2.0 * math.sqrt(max(vxx, 1e-4)))
                e.scale.y = float(2.0 * math.sqrt(max(vyy, 1e-4)))
                e.scale.z = 0.02
                frac = i / max(1, FORECAST_STEPS - 1)
                e.color.r = 1.0; e.color.g = float(frac); e.color.b = 0.0
                e.color.a = float(0.5 * (1.0 - 0.6 * frac))
                e.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                arr.markers.append(e)
        if arr.markers:
            self.pred_pub.publish(arr)


def main():
    rclpy.init()
    node = WorldModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
