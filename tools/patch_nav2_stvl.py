#!/usr/bin/env python3
"""
patch_nav2_stvl.py -- forecasts go into their own costmap layer that FORGETS old marks.

Problem: the forecast cloud (/predicted_obstacles) was a source of the normal ObstacleLayer. That layer keeps
a marked cell until a LiDAR ray passes through it again, so old forecasts and people who walked away left
"ghost" obstacles along their paths.

Fix: a Spatio-Temporal Voxel Layer (STVL) only for the forecast, in both costmaps. Every mark decays after
VOXEL_DECAY seconds, so the costmap always shows the current forecast and nothing older. The LiDAR stays in
the normal obstacle layer.

Needs:  sudo apt install ros-jazzy-spatio-temporal-voxel-layer
Run:    python3 ~/autonomy/tools/patch_nav2_stvl.py      (backup: nav2_params.yaml.prestvl.bak, safe to run again)
"""
import copy
import os
import shutil

import yaml

VOXEL_DECAY = 1.0   # s a forecast mark lives (the forecast is republished several times per second)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
P = os.path.join(ROOT, "src", "my_robot", "config", "nav2_params.yaml")

PREDICTION_LAYER = {
    "plugin": "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer",
    "enabled": True,
    "voxel_decay": VOXEL_DECAY,
    "decay_model": 0,                # 0 = linear: gone after voxel_decay seconds
    "voxel_size": 0.05,
    "track_unknown_space": True,
    "mark_threshold": 0,
    "update_footprint_enabled": False,
    "combination_method": 1,         # 1 = keep the maximum cost of this layer and the layers below
    "origin_z": 0.0,
    "publish_voxel_map": False,
    "transform_tolerance": 0.2,
    "mapping_mode": False,
    "map_save_duration": 60.0,
    "observation_sources": "predicted",
    "predicted": {
        "data_type": "PointCloud2",
        "topic": "/predicted_obstacles",
        "marking": True,
        "clearing": False,
        "obstacle_range": 40.0,      # cloud is in the map frame: range counts from the map origin
        "min_obstacle_height": 0.0,
        "max_obstacle_height": 2.0,
        "expected_update_rate": 0.0,
        "observation_persistence": 0.0,
        "inf_is_valid": False,
        "voxel_filter": False,
        "clear_after_reading": True,
    },
}

d = yaml.safe_load(open(P))
bak = P + ".prestvl.bak"
if not os.path.exists(bak):
    shutil.copyfile(P, bak)
    print("backup:", bak)

for name in ("local_costmap", "global_costmap"):
    c = d[name][name]["ros__parameters"]
    ob = c["obstacle_layer"]
    ob["observation_sources"] = "scan"
    ob.pop("predicted", None)
    c["prediction_layer"] = copy.deepcopy(PREDICTION_LAYER)   # deep copy: shared dicts would become YAML aliases
    plugins = [p for p in c["plugins"] if p != "prediction_layer"]
    i = plugins.index("inflation_layer") if "inflation_layer" in plugins else len(plugins)
    plugins.insert(i, "prediction_layer")
    c["plugins"] = plugins
    print("%s plugins: %s" % (name, plugins))

class NoAliasDumper(yaml.SafeDumper):
    """ROS 2's params parser rejects YAML aliases (&id001 / *id001), so never write them."""
    def ignore_aliases(self, data):
        return True


yaml.dump(d, open(P, "w"), Dumper=NoAliasDumper, sort_keys=False, default_flow_style=None)
print("forecast marks now decay after %.1f s; written: %s" % (VOXEL_DECAY, P))