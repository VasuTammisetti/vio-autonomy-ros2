#!/usr/bin/env python3
"""
person_mover -- makes 'person_standing' walk back and forth in Gazebo,
so the world model has a MOVING target to forecast (the centerpiece).

Uses `gz service` to set the model pose each tick along a straight path
that oscillates in y (walks side to side in front of the robot).
No ROS deps needed; pure Gazebo transport via the gz CLI.
"""
import subprocess
import time
import math

WORLD = "my_world"
NAME = "person_standing"
X_FIXED = 1.5          # stays ~1.5 m in front of the robot
Y_AMPLITUDE = 1.5      # walks from y=-1.5 to y=+1.5
PERIOD = 12.0          # seconds for a full back-and-forth cycle
RATE_HZ = 20.0
YAW = 3.14159          # face the robot


def set_pose(x, y, yaw):
    # Gazebo Sim set_pose service (Ionic/Harmonic): gz.msgs.Pose request
    req = (f'name: "{NAME}", position: {{x: {x}, y: {y}, z: 0.0}}, '
           f'orientation: {{x: 0.0, y: 0.0, z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
    cmd = [
        "gz", "service", "-s", f"/world/{WORLD}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300",
        "--req", req,
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    print(f"person_mover: walking '{NAME}' back and forth "
          f"(x={X_FIXED}, y=+/-{Y_AMPLITUDE}, period={PERIOD}s). Ctrl+C to stop.")
    t0 = time.time()
    dt = 1.0 / RATE_HZ
    try:
        while True:
            t = time.time() - t0
            y = Y_AMPLITUDE * math.sin(2 * math.pi * t / PERIOD)
            set_pose(X_FIXED, y, YAW)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nperson_mover stopped.")


if __name__ == "__main__":
    main()