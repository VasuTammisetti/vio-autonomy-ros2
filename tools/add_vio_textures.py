#!/usr/bin/env python3
"""
add_vio_textures.py -- give the camera something to track, without changing what the LiDAR sees.

1. Generates high-contrast random-pattern textures (lots of corners for visual odometry):
     src/my_robot/worlds/textures/wall_{north,south,east,west}.png   (4096 x 342, matches a 12 m x 1 m wall)
     src/my_robot/worlds/textures/floor.png                          (2048 x 2048)
   Each wall gets a different pattern (different random seed) so walls don't look alike.
2. Patches src/my_robot/worlds/my_world.sdf:
     - paints each wall's visual with its texture (geometry untouched -> LiDAR and the saved map unchanged)
     - adds 'vio_floor': a 2 mm visual-only floor tile (no collision; below the LiDAR plane)
   A backup is written to my_world.sdf.previo.bak first. Safe to run twice (skips what is already done).

Run from anywhere:  python3 ~/autonomy/tools/add_vio_textures.py
"""
import os
import re
import shutil
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DIR = os.path.join(ROOT, "src", "my_robot", "worlds")
WORLD = os.path.join(WORLD_DIR, "my_world.sdf")
TEX_DIR = os.path.join(WORLD_DIR, "textures")
WALLS = ["wall_north", "wall_south", "wall_east", "wall_west"]


def save_png(path, rgb):
    try:
        from PIL import Image
        Image.fromarray(rgb).save(path)
    except ImportError:
        import cv2
        cv2.imwrite(path, rgb[:, :, ::-1])


def make_texture(w, h, seed, n_shapes, min_sz, max_sz):
    """Random overlapping rectangles and discs in strong colours = many sharp corners."""
    rng = np.random.default_rng(seed)
    img = np.empty((h, w, 3), dtype=np.uint8)
    img[:] = rng.integers(60, 200, 3)
    yy, xx = np.mgrid[0:max_sz, 0:max_sz]
    for _ in range(n_shapes):
        color = rng.integers(0, 256, 3).astype(np.uint8)
        sw = int(rng.integers(min_sz, max_sz))
        sh = int(rng.integers(min_sz, max_sz))
        x0 = int(rng.integers(0, w))
        y0 = int(rng.integers(0, h))
        x1, y1 = min(w, x0 + sw), min(h, y0 + sh)
        if x1 <= x0 or y1 <= y0:
            continue
        if rng.random() < 0.65:                       # rectangle
            img[y0:y1, x0:x1] = color
        else:                                         # disc
            r = min(sw, sh) / 2.0
            mask = (xx[: y1 - y0, : x1 - x0] - r) ** 2 + (yy[: y1 - y0, : x1 - x0] - r) ** 2 <= r * r
            img[y0:y1, x0:x1][mask] = color
    return img


def textured_material(tex):
    return ("<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
            "<specular>0.05 0.05 0.05 1</specular>"
            "<pbr><metal><albedo_map>textures/%s</albedo_map>"
            "<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr></material>" % tex)


FLOOR_MODEL = """
    <model name="vio_floor">
      <static>true</static>
      <pose>0 0 0.001 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>11.9 11.9 0.002</size></box></geometry>
          %s
        </visual>
      </link>
    </model>
""" % textured_material("floor.png")


def main():
    if not os.path.isfile(WORLD):
        raise SystemExit("World file not found: " + WORLD)

    # 1. textures
    os.makedirs(TEX_DIR, exist_ok=True)
    for i, wall in enumerate(WALLS):
        save_png(os.path.join(TEX_DIR, wall + ".png"),
                 make_texture(4096, 342, seed=100 + i, n_shapes=1800, min_sz=10, max_sz=110))
    save_png(os.path.join(TEX_DIR, "floor.png"),
             make_texture(2048, 2048, seed=200, n_shapes=3500, min_sz=10, max_sz=120))
    print("textures written to", TEX_DIR)

    # 2. patch the world
    sdf = open(WORLD).read()
    backup = WORLD + ".previo.bak"
    if not os.path.exists(backup):
        shutil.copyfile(WORLD, backup)
        print("backup:", backup)

    for wall in WALLS:
        m = re.search(r'<model name="%s">.*?</model>' % wall, sdf, re.S)
        if not m:
            print("WARNING: %s not found, skipped" % wall)
            continue
        block = m.group(0)
        if "albedo_map" in block:
            print("%s already textured, skipped" % wall)
            continue
        vis = re.search(r'<visual name="v">.*?</visual>', block, re.S)
        new_vis = re.sub(r'<material>.*?</material>', textured_material(wall + ".png"),
                         vis.group(0), count=1, flags=re.S)
        new_block = block.replace(vis.group(0), new_vis)
        sdf = sdf.replace(block, new_block)
        print("%s textured" % wall)

    if 'name="vio_floor"' in sdf:
        print("vio_floor already present, skipped")
    else:
        idx = sdf.rfind("</world>")
        sdf = sdf[:idx] + FLOOR_MODEL + "  " + sdf[idx:]
        print("vio_floor added")

    open(WORLD, "w").write(sdf)
    print("world updated:", WORLD)


if __name__ == "__main__":
    main()