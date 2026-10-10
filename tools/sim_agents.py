#!/usr/bin/env python3
"""
sim_agents -- moves the people and the tugger cart through the factory hall and publishes their
positions as detections on /fused_objects (the input of the world model).

  - Routes, speeds and pauses come from tools/factory_layout.py (AGENTS).
  - Everything runs on SIMULATION time, so speeds are right relative to the robot at any Gazebo speed.
  - Agents behave a bit like real ones: they stop when the robot is close in front of them.
    If the robot is still in the way after WAIT_BEFORE_BACKOFF seconds, the agent steps away from the
    robot until it is BACKOFF_CLEAR metres away, then drifts back onto its route. That breaks the stand-off
    "robot waits for person, person waits for robot".
  - People have priority at crossings: the cart waits before a crossing point while a person is near it.
    People stop if the cart is right in front of them (it is already in the crossing and will clear it).
  - Positions come from simulator ground truth plus NOISE_STD of Gaussian noise. In the full stack
    the camera+LiDAR fusion node publishes the same /fused_objects format.

Gazebo poses are set with one gz set_pose call per agent (in parallel) in a background thread.
It logs every 20 s how many pose updates reached Gazebo.

Run (Gazebo up):  python3 ~/autonomy/tools/sim_agents.py
"""
import math
import random
import subprocess
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray

from factory_layout import AGENTS, HALL_X, HALL_Y

WORLD = "my_world"
RATE_HZ = 10.0
NOISE_STD = 0.05
YIELD_ROBOT = {"person": 0.9, "cart": 1.6}  # stop when the robot is this close in front (m)
AHEAD_COS = 0.3                             # "in front" = within about 70 deg of the walking direction
PERSON_YIELD_CART = 1.4                     # people stop when the cart is this close in front (m)
PATH_CLEAR = {"person": 0.7, "cart": 1.0}   # also stop if the robot is this close to where the agent will be
LOOKAHEAD = 2.0                             # ... within the next LOOKAHEAD seconds of its route (any angle)
WAIT_BEFORE_BACKOFF = 3.0                   # s blocked by the robot before stepping back
BACKOFF_CLEAR = 2.0                         # step back until the robot is this far away (m)
BACKOFF_SPEED = 0.5                         # fraction of normal speed when stepping back
MAX_STEP_ASIDE = 6.0                        # m: how far an agent steps away from the robot at most
HALL_MARGIN = 0.6                           # m: stepping aside never goes closer than this to a wall
ZONE_ENTER = 1.8                            # cart: distance to a crossing point that counts as "in the crossing"
ZONE_PERSON = 3.0                           # cart waits before a crossing while a person is this close to it
ZONE_WAIT_MAX = 8.0                         # ... but after this long it goes if nobody is right at the crossing
ZONE_PERSON_HARD = 1.2                      # (m) never enter while a person is this close to the crossing point


def crossing_points(agents):
    """Where people's straight paths cross the cart's straight path (pedestrian priority zones)."""
    carts = [a for a in agents if a.kind == "cart"]
    pts = []
    for c in carts:
        for (cx0, cy0, cx1, cy1, _, _) in c.legs:
            for p in agents:
                if p.kind != "person":
                    continue
                for (px0, py0, px1, py1, _, _) in p.legs:
                    d = (cx1 - cx0) * (py1 - py0) - (cy1 - cy0) * (px1 - px0)
                    if abs(d) < 1e-9:
                        continue
                    t = ((px0 - cx0) * (py1 - py0) - (py0 - cy0) * (px1 - px0)) / d
                    u = ((px0 - cx0) * (cy1 - cy0) - (py0 - cy0) * (cx1 - cx0)) / d
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        pts.append((cx0 + t * (cx1 - cx0), cy0 + t * (cy1 - cy0)))
    uniq = []
    for q in pts:
        if all(math.hypot(q[0] - r[0], q[1] - r[1]) > 0.5 for r in uniq):
            uniq.append(q)
    return uniq


class Agent:
    def __init__(self, name, kind, speed, waypoints):
        self.name, self.kind, self.speed = name, kind, speed
        self.legs = []  # (x0, y0, x1, y1, move_time, dwell)
        n = len(waypoints)
        for i in range(n):
            x0, y0, _ = waypoints[i]
            x1, y1, dwell = waypoints[(i + 1) % n]
            self.legs.append((x0, y0, x1, y1, math.hypot(x1 - x0, y1 - y0) / speed, dwell))
        self.period = sum(l[4] + l[5] for l in self.legs)
        self.tau = 0.0
        self.ox, self.oy = 0.0, 0.0   # step-aside offset from the route (used to make room for the robot)
        self.blocked_for = 0.0
        self.moved = False            # did it advance in the last step (a waiting cart does not block people)
        self.zone_wait = 0.0          # how long the cart has waited before a crossing
        self.backing = False
        self.x, self.y, self.yaw, self.dir = waypoints[0][0], waypoints[0][1], 0.0, None
        self.update_pose()

    def pose_at(self, tau):
        """(x, y, yaw, unit direction or None while pausing) at loop time tau."""
        t = tau % self.period
        for (x0, y0, x1, y1, mt, dw) in self.legs:
            yaw = math.atan2(y1 - y0, x1 - x0)
            if t < mt:
                f = t / mt
                return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, yaw, (math.cos(yaw), math.sin(yaw))
            t -= mt
            if t < dw:
                return x1, y1, yaw, None
            t -= dw
        x0, y0 = self.legs[0][0], self.legs[0][1]
        return x0, y0, 0.0, None

    def update_pose(self):
        x, y, self.yaw, self.dir = self.pose_at(self.tau)
        self.x, self.y = x + self.ox, y + self.oy

    def ahead(self, ox, oy, lim):
        """True if (ox, oy) is closer than lim and roughly in front of this agent."""
        if self.dir is None:
            return False
        dx, dy = ox - self.x, oy - self.y
        d = math.hypot(dx, dy)
        return 1e-6 < d < lim and (dx * self.dir[0] + dy * self.dir[1]) / d > AHEAD_COS

    def robot_on_path(self, robot):
        """True if the robot stands close to the stretch of route this agent walks in the next LOOKAHEAD s.
        Catches the robot coming from the side, which the 'in front' cone misses."""
        if self.dir is None:
            return False
        lim = PATH_CLEAR[self.kind]
        for k in range(1, 9):
            x, y = self.pose_at(self.tau + LOOKAHEAD * k / 8.0)[:2]
            if math.hypot(robot[0] - x, robot[1] - y) < lim:
                return True
        return False

    def step(self, dt, robot, people, carts, zones):
        """Advance along the route unless something is in the way. robot: (x, y) or None.
        carts: (x, y, moving) of carts; people only wait for a cart that is actually moving."""
        self.moved = False
        if self.backing:
            d = math.hypot(robot[0] - self.x, robot[1] - self.y) if robot is not None else 99.0
            if d > BACKOFF_CLEAR:
                self.backing = False
                self.blocked_for = 0.0
            else:
                # step straight away from the robot (not back along the route, which can turn towards it)
                ux, uy = (self.x - robot[0]) / max(d, 1e-6), (self.y - robot[1]) / max(d, 1e-6)
                step = self.speed * BACKOFF_SPEED * dt
                nx, ny = self.ox + ux * step, self.oy + uy * step
                bx, by = self.pose_at(self.tau)[:2]
                hx, hy = HALL_X / 2 - HALL_MARGIN, HALL_Y / 2 - HALL_MARGIN
                nx = min(max(bx + nx, -hx), hx) - bx   # stay inside the hall
                ny = min(max(by + ny, -hy), hy) - by
                if math.hypot(nx, ny) <= MAX_STEP_ASIDE:
                    self.ox, self.oy = nx, ny
                self.update_pose()
                return
        if robot is not None and (self.ahead(robot[0], robot[1], YIELD_ROBOT[self.kind]) or self.robot_on_path(robot)):
            self.blocked_for += dt
            if self.blocked_for > WAIT_BEFORE_BACKOFF:
                self.backing = True
            return
        self.blocked_for = 0.0
        if self.kind == "person" and any(mv and self.ahead(cx, cy, PERSON_YIELD_CART) for cx, cy, mv in carts):
            return
        if self.kind == "cart":
            nx, ny = self.pose_at(self.tau + dt)[:2]
            for zx, zy in zones:   # do not enter a crossing while a person is near it
                entering = math.hypot(self.x - zx, self.y - zy) > ZONE_ENTER >= math.hypot(nx - zx, ny - zy)
                if not entering:
                    continue
                near = [math.hypot(px - zx, py - zy) for px, py in people]
                hard = any(d < ZONE_PERSON_HARD for d in near)
                soft = any(d < ZONE_PERSON for d in near)
                if hard or (soft and self.zone_wait < ZONE_WAIT_MAX):
                    self.zone_wait += dt   # waiting at the crossing (gives up waiting after ZONE_WAIT_MAX)
                    return
            self.zone_wait = 0.0
        self.tau += dt
        self.moved = True
        k = max(0.0, 1.0 - 0.5 * dt)   # drift back onto the route over a few seconds
        self.ox, self.oy = self.ox * k, self.oy * k
        self.update_pose()


def pose_text(name, x, y, yaw):
    return ('name: "%s", position: {x: %.3f, y: %.3f, z: 0.0}, orientation: {x: 0, y: 0, z: %.4f, w: %.4f}'
            % (name, x, y, math.sin(yaw / 2), math.cos(yaw / 2)))


class PoseSender(threading.Thread):
    """Sends the latest poses to Gazebo as fast as the gz CLI allows.

    One gz set_pose call per agent, all started in parallel, then wait for all of them.
    (set_pose_vector exists in Gazebo, but our Pose_V request did not move the models, so it is not used.)
    """

    def __init__(self, logger):
        super().__init__(daemon=True)
        self.logger = logger
        self.lock = threading.Lock()
        self.poses = None
        self.running = True
        self.sent, self.failed = 0, 0
        self.last_report = time.time()

    def set(self, poses):
        with self.lock:
            self.poses = list(poses)

    def run(self):
        while self.running:
            with self.lock:
                poses = self.poses
            if not poses:
                time.sleep(0.05)
                continue
            procs = [subprocess.Popen(["gz", "service", "-s", "/world/%s/set_pose" % WORLD, "--reqtype", "gz.msgs.Pose",
                                       "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", pose_text(*p)],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True) for p in poses]
            for pr in procs:
                out, _ = pr.communicate()
                if pr.returncode == 0 and "true" in out:
                    self.sent += 1
                else:
                    self.failed += 1
            if time.time() - self.last_report > 20.0:
                self.logger.info("pose updates to Gazebo: %d ok, %d failed (last 20 s)" % (self.sent, self.failed))
                self.sent, self.failed = 0, 0
                self.last_report = time.time()


class SimAgents(Node):
    def __init__(self):
        super().__init__("sim_agents", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.agents = [Agent(*a) for a in AGENTS]
        self.zones = crossing_points(self.agents)
        self.robot = None
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 10)
        self.pub = self.create_publisher(MarkerArray, "/fused_objects", 10)
        self.sender = PoseSender(self.get_logger())
        self.sender.start()
        self.last = None
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info("sim_agents: moving %s on sim time"
                               % ", ".join("%s (%.1f m/s)" % (a.name, a.speed) for a in self.agents))

    def on_odom(self, msg):
        # odom starts at the robot spawn = world origin; drift over a run is small for this purpose
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last is None else min(max(now - self.last, 0.0), 0.5)
        self.last = now
        people = [(a.x, a.y) for a in self.agents if a.kind == "person"]
        carts = [(a.x, a.y, a.moved) for a in self.agents if a.kind == "cart"]
        for a in self.agents:
            a.step(dt, self.robot, people, carts, self.zones)
        self.sender.set([(a.name, a.x, a.y, a.yaw) for a in self.agents])
        self.publish()

    def publish(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, a in enumerate(self.agents):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = a.name
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = a.x + random.gauss(0.0, NOISE_STD)
            m.pose.position.y = a.y + random.gauss(0.0, NOISE_STD)
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35 if a.kind == "person" else 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 0.4, 1.0, 0.9) if a.kind == "person" else (1.0, 0.6, 0.0, 0.9)
            m.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns = a.name + "_label"
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = a.x, a.y, 1.0
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = a.kind
            t.lifetime = m.lifetime
            arr.markers.append(t)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = SimAgents()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sender.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()