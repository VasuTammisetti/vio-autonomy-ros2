#!/usr/bin/env python3
"""
patch_nav2_params.py -- updates src/my_robot/config/nav2_params.yaml for the factory hall.

  1. Global costmap also reads /predicted_obstacles, so the PLANNER routes around where people and the
     cart are heading (until now only the local controller saw the forecast).
  2. The prediction cloud is in the map frame, so Nav2 measures its "range" from the map origin.
     obstacle/raytrace ranges for it go to 40 m, otherwise forecasts more than 8 m from the hall
     centre would be dropped in the 20 x 15 m hall.
  3. Global costmap LiDAR ranges 15 m / 20 m (the LiDAR now reaches 20 m).
  4. Local costmap 3 x 3 m -> 5 x 5 m, so the controller sees forecasts a bit further ahead.

Backup on first run: nav2_params.yaml.prefactory.bak. Comments in the YAML are not kept (backup has them).
Run:  python3 ~/autonomy/tools/patch_nav2_params.py
"""
import copy
import os
import shutil

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
P = os.path.join(ROOT, "src", "my_robot", "config", "nav2_params.yaml")

d = yaml.safe_load(open(P))
bak = P + ".prefactory.bak"
if not os.path.exists(bak):
    shutil.copyfile(P, bak)
    print("backup:", bak)

loc = d["local_costmap"]["local_costmap"]["ros__parameters"]
glo = d["global_costmap"]["global_costmap"]["ros__parameters"]
lob, gob = loc["obstacle_layer"], glo["obstacle_layer"]

if "predicted" not in lob:
    raise SystemExit("local costmap has no 'predicted' source; send this output to Claude")

lob["predicted"]["obstacle_max_range"] = 40.0
lob["predicted"]["raytrace_max_range"] = 40.0
gob["observation_sources"] = "scan predicted"
gob["predicted"] = copy.deepcopy(lob["predicted"])
gob["scan"]["obstacle_max_range"] = 15.0
gob["scan"]["raytrace_max_range"] = 20.0
loc["width"] = 5
loc["height"] = 5

yaml.safe_dump(d, open(P, "w"), sort_keys=False, default_flow_style=None)
print("global costmap sources:", gob["observation_sources"])
print("predicted source:", gob["predicted"])
print("local costmap: %d x %d m" % (loc["width"], loc["height"]))
print("written:", P)
