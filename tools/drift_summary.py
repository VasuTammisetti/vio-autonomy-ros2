#!/usr/bin/env python3
"""
drift_summary.py -- one table from all drift test logs (~/autonomy/logs/drift_*.csv).

Why a max column: the square is a closed loop. If the heading is right but the wheels are 10% long,
the robot drives 10% short on every side in the same way, so it still ends up back at the start
and the end-point error looks small. The worst error along the way (usually at the far corner)
shows the distance error honestly.

Run from anywhere:  python3 ~/autonomy/tools/drift_summary.py
"""
import csv
import glob
import math
import os

LOG_DIR = os.path.expanduser("~/autonomy/logs")

rows = []
for path in sorted(glob.glob(os.path.join(LOG_DIR, "drift_*.csv"))):
    label = os.path.basename(path)[len("drift_"):-len(".csv")]
    data = [r for r in csv.DictReader(open(path))]
    if not data:
        continue
    x0, y0, yaw0 = float(data[0]["drift_x"]), float(data[0]["drift_y"]), float(data[0]["drift_yaw_deg"])
    pos = [math.hypot(float(r["drift_x"]) - x0, float(r["drift_y"]) - y0) for r in data]
    yaw = [float(r["drift_yaw_deg"]) - yaw0 for r in data]
    k = max(range(len(pos)), key=lambda i: pos[i])
    rows.append((label, pos[-1], max(pos), data[k]["phase"], yaw[-1], max(yaw, key=abs)))

if not rows:
    raise SystemExit("no drift_*.csv files in " + LOG_DIR)
print("%-18s %10s %10s %8s %11s %11s" % ("label", "end pos m", "max pos m", "at", "end yaw deg", "max yaw deg"))
for r in rows:
    print("%-18s %10.3f %10.3f %8s %11.2f %11.2f" % r)
print("\n12 m driven per run. Turn 4 is not in the numbers: slam_toolbox only updates while driving.")