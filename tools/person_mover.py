#!/usr/bin/env python3
"""
person_mover v2 -- walks 'person_standing' across the OPEN CENTER of the room
so its predicted path crosses where the robot navigates (isolating
prediction-avoidance from wall-avoidance).

Walks along a line through the middle of the room. Adjust WALK_AXIS / limits
to match your open space.
"""
import subprocess
import time
import math

WORLD = "my_world"
NAME = "person_standing"

# Walk the person back and forth through the room centre.
# Default: move along X (front-back) at y=0, through the open middle.
WALK_AXIS = "x"        # "x" = walk front/back, "y" = walk left/right
CENTER = 3.5           # fixed coordinate on the OTHER axis (keep in open space)
AMPLITUDE = 2.0        # how far each way from centre (keep inside walls ~<2.5)
PERIOD = 10.0          # seconds per full back-and-forth
RATE_HZ = 20.0


def quat_yaw(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def set_pose(x, y, yaw):
    z, w = quat_yaw(yaw)
    req = (f'name: "{NAME}", position: {{x: {x}, y: {y}, z: 0.0}}, '
           f'orientation: {{x: 0.0, y: 0.0, z: {z}, w: {w}}}')
    subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
         "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
         "--timeout", "300", "--req", req],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    print(f"person_mover v2: walking '{NAME}' along {WALK_AXIS} "
          f"(+/-{AMPLITUDE} about {CENTER}), period={PERIOD}s. Ctrl+C to stop.")
    t0 = time.time()
    dt = 1.0 / RATE_HZ
    try:
        while True:
            t = time.time() - t0
            offset = AMPLITUDE * math.sin(2 * math.pi * t / PERIOD)
            # velocity sign -> face walking direction
            vel = math.cos(2 * math.pi * t / PERIOD)
            if WALK_AXIS == "x":
                x, y = offset, CENTER
                yaw = 0.0 if vel >= 0 else math.pi
            else:
                x, y = CENTER, offset
                yaw = math.pi / 2 if vel >= 0 else -math.pi / 2
            set_pose(x, y, yaw)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nperson_mover stopped.")


if __name__ == "__main__":
    main()