#!/usr/bin/env python3
"""
make_factory_world.py -- turn the test room into a 20 x 15 m shared factory hall
(people, a tugger cart and the robot share the floor).

Layout (all numbers in tools/factory_layout.py):
  - main aisle east-west through the centre, robot starts in it at (0, 0)
  - zebra crossing with stop lines across the main aisle, walkway lines north and south of it
  - tugger cart lane (orange) in the south half, with its own small crossing
  - north-west: assembly benches     north-middle: parts rack     north-east: charging dock
  - south-west: fenced robot cell    south-east: 3 marked loading bays (2 with pallets)
  - 3 concrete pillars with unique IDs, 2.5 m walls with zone signs that are different on every wall
  - moving agents: person_standing + ped_2 + ped_3 (same person model) and a tugger cart;
    tools/sim_agents.py moves them

Floor markings are thin coloured boxes, not painted into the floor texture, so they sit exactly
where the layout says regardless of how Gazebo maps textures.

Backup on first run: my_world.sdf.prefactory.bak. Safe to run again.
Needs tools/make_warehouse_world.py next to it (shared texture helpers).

Run:  python3 ~/autonomy/tools/make_factory_world.py
"""
import math
import os
import re
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

import make_warehouse_world as wh
from factory_layout import (AGENTS, CART_LANE_HALF, CART_LANE_Y, CROSSING_X, HALL_X, HALL_Y,
                            MAIN_AISLE_HALF, PILLARS, STATIONS, WALL_H)
from make_warehouse_world import blotches, cardboard, draw_sign, font, mat, pallet_wood, text_center, to_img

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DIR = os.path.join(ROOT, "src", "my_robot", "worlds")
WORLD = os.path.join(WORLD_DIR, "my_world.sdf")
TEX = os.path.join(WORLD_DIR, "textures")
PPM_WALL = 205  # wall texture pixels per metre

HX, HY = HALL_X / 2, HALL_Y / 2
T = 0.1  # wall thickness
# wall name -> (pose x, y, yaw, length, direction)
WALL_GEOM = {
    "wall_north": (0.0, HY + T / 2, 0.0, HALL_X + 2 * T, "north"),
    "wall_south": (0.0, -HY - T / 2, 0.0, HALL_X + 2 * T, "south"),
    "wall_east": (HX + T / 2, 0.0, 1.5708, HALL_Y + 2 * T, "east"),
    "wall_west": (-HX - T / 2, 0.0, 1.5708, HALL_Y + 2 * T, "west"),
}


def wall_u(direction, x=None, y=None):
    """0..1 position along a wall texture, measured from the left as seen from inside the hall."""
    L = {"north": HALL_X, "south": HALL_X, "east": HALL_Y, "west": HALL_Y}[direction] + 2 * T
    if direction == "north":
        d = x + L / 2
    elif direction == "south":
        d = L / 2 - x
    elif direction == "east":
        d = L / 2 - y
    else:
        d = y + L / 2
    return d / L


# zone signs at fixed places: (wall, x or y along the wall, title, subtitle, colour)
ZONE_SIGNS = [
    ("north", -6.8, "ASSEMBLY", "LINE 2", (30, 90, 170)),
    ("north", -0.5, "PARTS", "RACK P4", (90, 90, 100)),
    ("north", 2.5, "WALKWAY", "PEDESTRIANS", (20, 130, 70)),
    ("east", 4.5, "CHARGING", "AMR DOCK", (20, 130, 70)),
    ("east", -3.2, "TUGGER", "LANE T1", (220, 110, 20)),
    ("south", 4.0, "BAY 1", "LOADING", (25, 70, 160)),
    ("south", 6.0, "BAY 2", "LOADING", (25, 70, 160)),
    ("south", 8.0, "BAY 3", "LOADING", (25, 70, 160)),
    ("south", -8.5, "ROBOT CELL", "KEEP OUT", (200, 40, 35)),
    ("west", -6.0, "QA LAB", "CELL 1", (120, 60, 150)),
    ("west", 4.8, "TOOLS", "STATION", (90, 90, 100)),
]


# ----------------------------------------------------------------------------- textures
def hall_floor(size=2048):
    rng = np.random.default_rng(21)
    base = 140 + blotches(size, size, rng, (30, 30), 14) + blotches(size, size, rng, (200, 200), 5)
    base += rng.normal(0, 4, (size, size))
    img = to_img(np.stack([base, base * 0.99, base * 0.965], axis=-1))
    stain = Image.new("L", (size, size), 0)
    sd = ImageDraw.Draw(stain)
    for _ in range(70):
        cx, cy = rng.integers(0, size, 2)
        rx, ry = rng.integers(8, 60, 2)
        sd.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=int(rng.integers(25, 75)))
    for _ in range(14):  # tyre marks
        x0, y0 = rng.integers(0, size, 2)
        ang = rng.uniform(0, math.pi)
        ln = rng.integers(150, 500)
        x1, y1 = x0 + ln * math.cos(ang), y0 + ln * math.sin(ang)
        sd.line([(x0, y0), (x1, y1)], fill=40, width=6)
        sd.line([(x0 + 18, y0 + 18), (x1 + 18, y1 + 18)], fill=40, width=6)
    stain = stain.filter(ImageFilter.GaussianBlur(6))
    img = Image.composite(Image.new("RGB", (size, size), (80, 78, 74)), img, stain)
    d = ImageDraw.Draw(img)
    for k in range(1, 8):  # slab joints
        p = int(k * size / 8)
        d.line([(p, 0), (p, size)], fill=(92, 90, 86), width=3)
        d.line([(0, p), (size, p)], fill=(92, 90, 86), width=3)
    return img


def hall_wall(direction, seed):
    rng = np.random.default_rng(seed)
    L = {"north": HALL_X, "south": HALL_X, "east": HALL_Y, "west": HALL_Y}[direction] + 2 * T
    w, h = int(L * PPM_WALL), int(WALL_H * PPM_WALL)
    ppm = PPM_WALL

    def yy(m):  # height above floor (m) -> image row
        return int(h - m * ppm)

    base = 214 + blotches(w, h, rng, (80, 8), 6) + rng.normal(0, 2.0, (h, w))
    img = to_img(np.stack([base * 0.98, base, base * 1.02], axis=-1))
    d = ImageDraw.Draw(img)
    # cladding panel seams every 1.25 m, horizontal joint at 1.25 m
    x = 1.25
    while x < L:
        p = int(x * ppm)
        d.line([(p, 0), (p, h)], fill=(160, 162, 166), width=3)
        d.line([(p + 3, 0), (p + 3, h)], fill=(232, 234, 238), width=1)
        x += 1.25
    d.line([(0, yy(1.25)), (w, yy(1.25))], fill=(170, 172, 176), width=2)
    # high windows strip 1.95-2.35 m with mullions
    d.rectangle([0, yy(2.35), w, yy(1.95)], fill=(150, 180, 205))
    x = 0.0
    while x < L:
        d.rectangle([int(x * ppm), yy(2.35), int(x * ppm) + 10, yy(1.95)], fill=(90, 95, 100))
        x += 1.0
    # cable tray at 1.85 m
    d.rectangle([0, yy(1.88), w, yy(1.82)], fill=(120, 124, 128))
    # dark lower band 0.15-0.55 m and hazard kick plate 0-0.15 m
    d.rectangle([0, yy(0.55), w, yy(0.15)], fill=(96, 100, 106))
    kick_top = yy(0.15)
    stripe = int(0.12 * ppm)
    hz = Image.new("RGB", (w, h - kick_top), (25, 25, 25))
    hd = ImageDraw.Draw(hz)
    for sx in range(-80, w + 80, stripe * 2):
        hd.polygon([(sx, h - kick_top), (sx + stripe, h - kick_top), (sx + stripe + 30, 0), (sx + 30, 0)], fill=(240, 196, 25))
    img.paste(hz, (0, kick_top))
    d = ImageDraw.Draw(img)

    placed = []  # (x0, x1) in pixels, keeps signs from overlapping
    # zone signs at their fixed places, 1.3-1.75 m high
    for (wd, pos, t1, t2, col) in ZONE_SIGNS:
        if wd != direction:
            continue
        u = wall_u(direction, x=pos, y=pos)
        sw, sh = int(1.4 * ppm), int(0.45 * ppm)
        sx = int(u * w - sw / 2)
        sy = yy(1.75)
        d.rectangle([sx, sy, sx + sw, sy + sh], fill=col, outline=(255, 255, 255), width=6)
        text_center(d, (sx, sy + 4, sx + sw, sy + sh * 0.62), t1, int(sh * 0.42), (255, 255, 255))
        text_center(d, (sx, sy + sh * 0.6, sx + sw, sy + sh - 4), t2, int(sh * 0.24), (255, 255, 255))
        placed.append((sx, sx + sw))
    # generic safety signs and posters, 0.6-1.2 m high, unique numbers per wall
    signs = list(wh.SIGNS) + [("warn", "AUTOMATED", "VEHICLES"), ("poster", "SHIFT PLAN", ""), ("poster", "5S BOARD", "")]
    order = rng.permutation(len(signs))
    n, target = 0, int(L / 1.6)
    for idx in order:
        if n >= target:
            break
        kind, t1, t2 = signs[idx]
        if kind == "aisle":
            t2 = {"AISLE": "%s%d" % ("JKLMNPQR"[seed % 8], int(rng.integers(1, 10))),
                  "ZONE": str(int(rng.integers(10, 40))), "DOCK": str(int(rng.integers(13, 30)))}.get(t1, t2)
        sw = int((0.62 if kind in ("exit", "aisle") else 0.45) * ppm)
        sh = int((0.3 if kind == "exit" else 0.55) * ppm)
        for _ in range(80):
            sx = int(rng.integers(30, w - sw - 30))
            if all(sx + sw + 50 < a or sx > b + 50 for a, b in placed):
                sy = yy(1.2) + int(rng.integers(0, max(1, yy(0.62) - yy(1.2) - sh)))
                draw_sign(img, kind, t1, t2, sx, sy, sw, sh, rng)
                placed.append((sx, sx + sw))
                n += 1
                break
    # a few fixed items low on the wall (fire hose box, sockets) for close-range texture
    for _ in range(int(L / 4)):
        sx = int(rng.integers(40, w - 200))
        if all(sx + 120 < a or sx > b for a, b in placed):
            d.rectangle([sx, yy(0.95), sx + 110, yy(0.65)], fill=(190, 40, 40), outline=(60, 20, 20), width=3)
            d.rectangle([sx + 15, yy(0.9), sx + 95, yy(0.72)], fill=(230, 230, 230))
            placed.append((sx, sx + 110))
    if wh.FLIP_V:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
    if wh.FLIP_H:
        img = img.transpose(Image.FLIP_LEFT_RIGHT)
    return img


def fence_mesh(w=1024, h=512):
    img = Image.new("RGB", (w, h), (175, 180, 172))
    d = ImageDraw.Draw(img)
    for x in range(0, w, 22):
        d.line([(x, 0), (x, h)], fill=(70, 74, 70), width=3)
    for y in range(0, h, 22):
        d.line([(0, y), (w, y)], fill=(70, 74, 70), width=3)
    d.rectangle([w // 2 - 170, h // 2 - 60, w // 2 + 170, h // 2 + 60], fill=(250, 210, 30), outline=(20, 20, 20), width=5)
    text_center(d, (w // 2 - 170, h // 2 - 60, w // 2 + 170, h // 2 + 60), "DANGER ROBOT", 40, (20, 20, 20))
    return img


def pillar_tex(label, seed, w=256, h=1280):
    rng = np.random.default_rng(seed)
    base = 175 + blotches(w, h, rng, (8, 20), 10) + rng.normal(0, 4, (h, w))
    img = to_img(np.stack([base, base * 0.99, base * 0.96], axis=-1))
    d = ImageDraw.Draw(img)
    band = int(h * 0.8 / 2.5)  # hazard band 0-0.8 m
    stripe = 40
    for sx in range(-h, w + h, stripe * 2):
        d.polygon([(sx, h), (sx + stripe, h), (sx + stripe + band, h - band), (sx + band, h - band)], fill=(240, 196, 25))
    hz = Image.new("RGB", (w, band), (25, 25, 25))
    hd = ImageDraw.Draw(hz)
    for sx in range(-band, w + band, stripe * 2):
        hd.polygon([(sx, band), (sx + stripe, band), (sx + stripe + band, 0), (sx + band, 0)], fill=(240, 196, 25))
    img.paste(hz, (0, h - band))
    d = ImageDraw.Draw(img)
    d.rectangle([28, int(h * 0.3), w - 28, int(h * 0.3) + 120], fill=(25, 70, 160), outline=(255, 255, 255), width=5)
    text_center(d, (28, int(h * 0.3), w - 28, int(h * 0.3) + 120), label, 84, (255, 255, 255))
    return img


def bench_front(w=1024, h=512):
    img = Image.new("RGB", (w, h), (60, 95, 140))
    d = ImageDraw.Draw(img)
    for c in range(3):
        x0 = 20 + c * (w - 40) // 3
        x1 = x0 + (w - 40) // 3 - 16
        for r in range(4):
            y0 = 30 + r * (h - 60) // 4
            y1 = y0 + (h - 60) // 4 - 12
            d.rectangle([x0, y0, x1, y1], fill=(72, 112, 160), outline=(35, 55, 85), width=4)
            d.rectangle([(x0 + x1) // 2 - 50, y0 + 18, (x0 + x1) // 2 + 50, y0 + 32], fill=(200, 200, 205))
            d.rectangle([x0 + 14, y0 + 40, x0 + 90, y0 + 62], fill=(245, 245, 240))
    return img


def dock_tex(w=640, h=400):
    img = Image.new("RGB", (w, h), (55, 58, 64))
    d = ImageDraw.Draw(img)
    d.rectangle([30, 30, 250, 300], fill=(30, 32, 36), outline=(20, 160, 80), width=8)
    d.polygon([(165, 60), (85, 170), (140, 170), (115, 270), (205, 140), (150, 140)], fill=(40, 200, 90))
    text_center(d, (270, 60, w - 20, 180), "CHARGE", 72, (230, 230, 230))
    text_center(d, (270, 180, w - 20, 260), "AMR 01", 54, (180, 180, 180))
    d.rectangle([30, 330, w - 30, 360], fill=(40, 200, 90))
    return img


def cart_tex(w=1024, h=384):
    img = Image.new("RGB", (w, h), (238, 180, 20))
    d = ImageDraw.Draw(img)
    for sx in range(-h, w + h, 80):
        d.polygon([(sx, h), (sx + 40, h), (sx + 40 + 70, h - 70), (sx + 70, h - 70)], fill=(25, 25, 25))
    text_center(d, (40, 50, w - 170, 220), "TUGGER T07", 92, (25, 25, 25))
    d.ellipse([w - 140, 80, w - 60, 160], fill=(230, 60, 40), outline=(25, 25, 25), width=4)
    return img


# ----------------------------------------------------------------------------- SDF helpers
def color_mat(r, g, b):
    return ("<material><ambient>%.2f %.2f %.2f 1</ambient><diffuse>%.2f %.2f %.2f 1</diffuse>"
            "<specular>0.05 0.05 0.05 1</specular></material>" % (r, g, b, r, g, b))


def box(name, x, y, z, sx, sy, sz, material, yaw=0.0, collide=True):
    pose = "%.3f %.3f %.3f 0 0 %.4f" % (x, y, z, yaw)
    geo = "<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>" % (sx, sy, sz)
    s = '        <visual name="v_%s"><pose>%s</pose>%s%s</visual>\n' % (name, pose, geo, material)
    if collide:
        s += '        <collision name="c_%s"><pose>%s</pose>%s</collision>\n' % (name, pose, geo)
    return s


def collision(name, x, y, z, sx, sy, sz, yaw=0.0):
    return ('        <collision name="c_%s"><pose>%.3f %.3f %.3f 0 0 %.4f</pose><geometry><box><size>%.3f %.3f %.3f</size>'
            '</box></geometry></collision>\n' % (name, x, y, z, yaw, sx, sy, sz))


def cyl(name, x, y, z, r, ln, material, collide=True):
    pose = "%.3f %.3f %.3f 0 0 0" % (x, y, z)
    geo = "<geometry><cylinder><radius>%.3f</radius><length>%.3f</length></cylinder></geometry>" % (r, ln)
    s = '        <visual name="v_%s"><pose>%s</pose>%s%s</visual>\n' % (name, pose, geo, material)
    if collide:
        s += '        <collision name="c_%s"><pose>%s</pose>%s</collision>\n' % (name, pose, geo)
    return s


def model(name, x, y, yaw, body, static=True):
    return ('    <model name="%s">\n      <static>%s</static>\n      <pose>%.3f %.3f 0 0 0 %.4f</pose>\n'
            '      <link name="link">\n%s      </link>\n    </model>\n'
            % (name, "true" if static else "false", x, y, yaw, body))


WHITE = color_mat(0.93, 0.93, 0.9)
YELLOW = color_mat(0.9, 0.72, 0.1)
GREEN = color_mat(0.15, 0.6, 0.3)
ORANGE = color_mat(0.95, 0.45, 0.08)
STEEL = color_mat(0.45, 0.47, 0.5)
DARK = color_mat(0.12, 0.12, 0.13)
BLUE = color_mat(0.12, 0.35, 0.7)
ROBOT_ORANGE = color_mat(0.95, 0.5, 0.1)

MZ, MH = 0.004, 0.003  # markings: centre height and thickness (floor top is at 0.002)


def line(name, x0, y0, x1, y1, width, material):
    """Thin floor marking from (x0,y0) to (x1,y1)."""
    ln = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    return box(name, (x0 + x1) / 2, (y0 + y1) / 2, MZ, ln, width, MH, material, yaw=yaw, collide=False)


def dashed(name, x0, y0, x1, y1, width, material, dash=0.6, gap=0.4, skip=None):
    out, ln = "", math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / ln, (y1 - y0) / ln
    s, k = 0.0, 0
    while s < ln:
        e = min(s + dash, ln)
        ax, ay, bx, by = x0 + ux * s, y0 + uy * s, x0 + ux * e, y0 + uy * e
        if not (skip and skip(ax, ay) and skip(bx, by)):
            out += line("%s%d" % (name, k), ax, ay, bx, by, width, material)
        s += dash + gap
        k += 1
    return out


def markings():
    b = ""
    a = MAIN_AISLE_HALF
    cx = CROSSING_X
    # main aisle edges (yellow), interrupted at the zebra crossing
    for i, yv in enumerate((a, -a)):
        b += line("aisle_w%d" % i, -HX + 0.5, yv, cx - 1.05, yv, 0.08, YELLOW)
        b += line("aisle_e%d" % i, cx + 1.05, yv, HX - 0.5, yv, 0.08, YELLOW)
    # zebra crossing: bars along the aisle, stacked across it
    n = 0
    yv = -a + 0.2
    while yv + 0.4 <= a:
        b += box("zebra%d" % n, cx, yv + 0.2, MZ, 2.0, 0.4, MH, WHITE, collide=False)
        yv += 0.8
        n += 1
    # stop lines either side of the crossing
    for i, xv in enumerate((cx - 1.6, cx + 1.6)):
        b += line("stop%d" % i, xv, -a + 0.05, xv, a - 0.05, 0.14, WHITE)
    # pedestrian walkway lines (green) north and south of the crossing
    lane_n, lane_s = CART_LANE_Y + CART_LANE_HALF, CART_LANE_Y - CART_LANE_HALF
    for i, xv in enumerate((cx - 1.0, cx + 1.0)):
        b += line("walk_n%d" % i, xv, a, xv, HY - 0.6, 0.07, GREEN)
        b += line("walk_m%d" % i, xv, -a, xv, lane_n, 0.07, GREEN)
        b += line("walk_s%d" % i, xv, lane_s, xv, -5.05, 0.07, GREEN)
    # tugger lane (orange, dashed) with a small crossing where the walkway meets it
    in_walk = lambda px, py: abs(px - cx) < 1.05
    for i, yv in enumerate((lane_n, lane_s)):
        b += dashed("cart_l%d_" % i, -HX + 1.2, yv, HX - 0.6, yv, 0.07, ORANGE, skip=in_walk)
    for k in range(3):
        b += box("cart_zebra%d" % k, cx, lane_s + 0.2 + k * 0.4, MZ, 2.0, 0.22, MH, WHITE, collide=False)
    # loading bays along the south wall: x 3..9, depth 2.5 m
    for i, xv in enumerate((3.0, 5.0, 7.0, 9.0)):
        b += line("bay%d" % i, xv, -HY + 0.05, xv, -5.0, 0.1, WHITE)
    b += line("bay_front", 3.0, -5.0, 9.0, -5.0, 0.06, YELLOW)
    # charging box (green) in front of the dock
    sx, sy = 8.8, 4.5
    for i, (x0, y0, x1, y1) in enumerate([(sx - 0.6, sy - 0.6, HX - 0.05, sy - 0.6), (sx - 0.6, sy + 0.6, HX - 0.05, sy + 0.6),
                                          (sx - 0.6, sy - 0.6, sx - 0.6, sy + 0.6)]):
        b += line("chg%d" % i, x0, y0, x1, y1, 0.08, GREEN)
    # robot cell keep-out line 0.4 m outside the fence
    b += line("cell_n", -HX + 0.05, -4.2, -6.6, -4.2, 0.1, YELLOW)
    b += line("cell_e", -6.6, -4.2, -6.6, -HY + 0.05, 0.1, YELLOW)
    return model("fac_markings", 0, 0, 0, b)


def bench(name, x, y):
    b = box("cab", 0, 0, 0.42, 1.8, 0.72, 0.84, mat("bench_front.png"))
    b += box("top", 0, 0, 0.865, 1.9, 0.8, 0.05, color_mat(0.62, 0.48, 0.32), collide=False)
    b += box("mon", -0.3, 0.2, 1.12, 0.55, 0.05, 0.34, DARK, collide=False)
    b += box("mon_st", -0.3, 0.25, 0.95, 0.06, 0.06, 0.12, DARK, collide=False)
    b += box("bin1", 0.45, 0.05, 0.94, 0.3, 0.2, 0.12, BLUE, collide=False)
    b += box("bin2", 0.75, 0.05, 0.94, 0.25, 0.2, 0.12, color_mat(0.8, 0.15, 0.15), collide=False)
    b += box("board", 0, 0.37, 1.45, 1.7, 0.03, 0.6, mat("box_c.png"), collide=False)
    return model(name, x, y, 0.0, b)  # local -y = front (faces the hall)


def rack(name, x, y, length=3.0, depth=0.6, height=2.0, rng=None):
    b = ""
    nup = int(length / 1.0) + 1
    for i in range(nup):
        ux = -length / 2 + i * length / (nup - 1)
        for j, uy in enumerate((-depth / 2 + 0.03, depth / 2 - 0.03)):
            b += box("up%d_%d" % (i, j), ux, uy, height / 2, 0.06, 0.06, height, BLUE, collide=False)
    levels = [0.18, 0.85, 1.5]
    for k, z in enumerate(levels):
        b += box("beam%d" % k, 0, -depth / 2 + 0.03, z, length, 0.06, 0.1, ROBOT_ORANGE, collide=False)
        b += box("deck%d" % k, 0, 0, z + 0.06, length, depth, 0.02, STEEL, collide=False)
        xx = -length / 2 + 0.1
        m = 0
        while True:
            bw, bd, bh = wh.BOX_SIZES[int(rng.integers(0, len(wh.BOX_SIZES)))]
            if xx + bw > length / 2 - 0.05:
                break
            if rng.random() < 0.85:
                b += box("bx%d_%d" % (k, m), xx + bw / 2, 0, z + 0.07 + bh / 2, bw, bd, bh,
                         mat("box_%s.png" % "abc"[int(rng.integers(0, 3))]), collide=False)
            xx += bw + 0.05
            m += 1
    b += box("kick", 0, -depth / 2 + 0.01, 0.12, length, 0.02, 0.24, YELLOW, collide=False)
    b += collision("rack", 0, 0, height / 2, length, depth, height)
    return model(name, x, y, 0.0, b)  # local -y = front with beams and kick plate


def robot_cell():
    """Fence along x=-7.0 (y -7.45..-4.6) and y=-4.6 (x -9.95..-7.0), cobot stand-in inside."""
    b = ""
    fx, fy = -7.0, -4.6
    segs = [((fx, -HY + 0.05), (fx, fy)), ((-HX + 0.05, fy), (fx, fy))]
    for i, ((x0, y0), (x1, y1)) in enumerate(segs):
        ln = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        b += box("kick%d" % i, mx, my, 0.2, ln, 0.04, 0.3, YELLOW, yaw=yaw, collide=False)
        b += box("mesh%d" % i, mx, my, 1.1, ln, 0.02, 1.5, mat("fence_mesh.png"), yaw=yaw, collide=False)
        b += collision("fcol%d" % i, mx, my, 0.9, ln, 0.06, 1.8, yaw=yaw)
        n = int(ln / 1.2) + 1
        for k in range(n + 1):
            t = k / n
            b += box("post%d_%d" % (i, k), x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, 0.95, 0.06, 0.06, 1.9,
                     YELLOW, collide=False)
    # cobot stand-in: pedestal + arm links
    cx, cy = -8.6, -6.1
    b += cyl("ped", cx, cy, 0.4, 0.22, 0.8, STEEL, collide=False)
    b += cyl("j1", cx, cy, 0.88, 0.16, 0.16, ROBOT_ORANGE, collide=False)
    b += box("l1", cx + 0.25, cy, 1.05, 0.6, 0.14, 0.14, color_mat(0.9, 0.9, 0.9), collide=False)
    b += box("l2", cx + 0.55, cy + 0.15, 0.85, 0.12, 0.12, 0.45, color_mat(0.9, 0.9, 0.9), collide=False)
    b += box("tbl", cx + 0.6, cy + 0.6, 0.4, 0.9, 0.6, 0.8, STEEL, collide=False)
    return model("fac_robot_cell", 0, 0, 0, b)


def dock():
    b = box("body", 0, 0, 0.25, 0.4, 0.8, 0.5, mat("dock.png"))
    b += box("led", -0.205, 0, 0.42, 0.01, 0.6, 0.04, color_mat(0.2, 0.95, 0.4), collide=False)
    return model("fac_dock", HX - 0.25, 4.5, 0, b)


def pillar(name, x, y, label):
    return model(name, x, y, 0, box("p", 0, 0, WALL_H / 2, 0.5, 0.5, WALL_H, mat("pillar_%s.png" % label)))


def cart_model():
    """Tugger cart: solid down to the floor so the LiDAR cannot see under it. Moved by sim_agents.py."""
    b = box("body", 0, 0, 0.2, 1.0, 0.62, 0.36, mat("cart_body.png"))
    b += box("deck", 0, 0, 0.4, 1.02, 0.64, 0.04, DARK, collide=False)
    b += box("tote1", -0.22, 0, 0.56, 0.5, 0.36, 0.28, BLUE, collide=False)
    b += box("tote2", 0.28, 0, 0.56, 0.4, 0.36, 0.28, color_mat(0.15, 0.55, 0.3), collide=False)
    b += box("mast", -0.45, 0, 0.75, 0.06, 0.06, 0.7, DARK, collide=False)
    b += box("lamp", -0.45, 0, 1.12, 0.1, 0.1, 0.06, color_mat(1.0, 0.55, 0.1), collide=False)
    b += collision("load", 0, 0, 0.55, 1.0, 0.62, 0.3)
    x, y = AGENTS[3][3][0][0], AGENTS[3][3][0][1]
    return model("cart", x, y, 0, b)


def floor_model():
    return model("hall_floor", 0, 0, 0, box("f", 0, 0, 0.001, HALL_X - 0.1, HALL_Y - 0.1, 0.002, mat("hall_floor.jpg"),
                                             collide=False))


# ----------------------------------------------------------------------------- patching
def remove_model(sdf, name_regex):
    return re.sub(r'\n?[ \t]*<model name="%s">.*?\n[ \t]*</model>[ \t]*' % name_regex, "", sdf, flags=re.S)


def remove_include(sdf, name):
    return re.sub(r'\n?[ \t]*<include>(?:(?!</include>).)*?<name>%s</name>.*?</include>[ \t]*' % re.escape(name), "", sdf,
                  flags=re.S)


def main():
    if not os.path.isfile(WORLD):
        raise SystemExit("World file not found: " + WORLD)
    os.makedirs(TEX, exist_ok=True)

    hall_floor().save(os.path.join(TEX, "hall_floor.jpg"), quality=88)   # jpg: 1 MB instead of 6 MB
    for f in ["floor_concrete.png", "wall_panel_north.png", "wall_panel_south.png", "wall_panel_east.png",
              "wall_panel_west.png", "hall_floor.png"]:   # warehouse textures, no longer used
        if os.path.exists(os.path.join(TEX, f)):
            os.remove(os.path.join(TEX, f))
    for i, (wall, g) in enumerate(WALL_GEOM.items()):
        hall_wall(g[4], seed=500 + i).save(os.path.join(TEX, "hall_wall_%s.png" % g[4]))
    for i, v in enumerate("abc"):
        cardboard(seed=i).save(os.path.join(TEX, "box_%s.png" % v))
    pallet_wood().save(os.path.join(TEX, "pallet_wood.png"))
    fence_mesh().save(os.path.join(TEX, "fence_mesh.png"))
    for k, (_, _, lab) in enumerate(PILLARS):
        pillar_tex(lab, 900 + k).save(os.path.join(TEX, "pillar_%s.png" % lab))
    bench_front().save(os.path.join(TEX, "bench_front.png"))
    dock_tex().save(os.path.join(TEX, "dock.png"))
    cart_tex().save(os.path.join(TEX, "cart_body.png"))
    print("textures written to", TEX)

    sdf = open(WORLD).read()
    backup = WORLD + ".prefactory.bak"
    if not os.path.exists(backup):
        shutil.copyfile(WORLD, backup)
        print("backup:", backup)

    # walls: new size, position and texture
    for wall, (x, y, yaw, ln, d) in WALL_GEOM.items():
        m = re.search(r'<model name="%s">.*?</model>' % wall, sdf, re.S)
        if not m:
            print("WARNING: %s not found, skipped" % wall)
            continue
        block = m.group(0)
        nb = re.sub(r"<pose>[^<]*</pose>", "<pose>%.3f %.3f %.3f 0 0 %.4f</pose>" % (x, y, WALL_H / 2, yaw), block, count=1)
        nb = re.sub(r"<size>[^<]*</size>", "<size>%.3f %.3f %.3f</size>" % (ln, T, WALL_H), nb)
        vis = re.search(r'<visual name="v">.*?</visual>', nb, re.S)
        if vis:
            nv = re.sub(r"<material>.*?</material>", mat("hall_wall_%s.png" % d), vis.group(0), count=1, flags=re.S)
            if "<material>" not in vis.group(0):
                nv = vis.group(0).replace("</visual>", mat("hall_wall_%s.png" % d) + "</visual>")
            nb = nb.replace(vis.group(0), nv)
        sdf = sdf.replace(block, nb)
        print("%s: %.1f m long, %.1f m high" % (wall, ln, WALL_H))

    # people: move the existing person to its start, add two copies of the same model
    inc = re.search(r"<include>(?:(?!</include>).)*?<name>person_standing</name>.*?</include>", sdf, re.S)
    if not inc:
        raise SystemExit("person_standing include not found in the world file")
    sdf = remove_include(sdf, "ped_2")
    sdf = remove_include(sdf, "ped_3")
    inc = re.search(r"<include>(?:(?!</include>).)*?<name>person_standing</name>.*?</include>", sdf, re.S).group(0)
    starts = {name: wps[0] for (name, kind, sp, wps) in AGENTS}
    new_inc = re.sub(r"<pose>[^<]*</pose>", "<pose>%.2f %.2f 0 0 0 0</pose>" % starts["person_standing"][:2], inc)
    sdf = sdf.replace(inc, new_inc)
    extra = ""
    for name in ("ped_2", "ped_3"):
        e = re.sub(r"<name>person_standing</name>", "<name>%s</name>" % name, new_inc)
        e = re.sub(r"<pose>[^<]*</pose>", "<pose>%.2f %.2f 0 0 0 0</pose>" % starts[name][:2], e)
        extra += "    " + e + "\n"

    # remove everything this script or the warehouse script added before
    for rx in ("vio_floor", r"prop_[a-z0-9]+", r"fac_[a-z0-9_]+", "hall_floor", "cart"):
        sdf = remove_model(sdf, rx)

    rng = np.random.default_rng(42)
    models = floor_model() + markings()
    models += bench("fac_bench_1", -8.3, HY - 0.48) + bench("fac_bench_2", -6.2, HY - 0.48)
    models += rack("fac_rack_p4", -0.5, HY - 0.36, rng=rng)
    models += robot_cell() + dock()
    for k, (px, py, lab) in enumerate(PILLARS):
        models += pillar("fac_pillar_%d" % (k + 1), px, py, lab)
    models += wh.prop_model("fac_bay2", 6.0, -6.4, 0.0, "pallet", rng)
    models += wh.prop_model("fac_bay3", 8.0, -6.4, 0.0, "pallet", rng)
    models += cart_model()
    idx = sdf.rfind("</world>")
    sdf = sdf[:idx].rstrip() + "\n" + extra + models + "  " + sdf[idx:]
    open(WORLD, "w").write(sdf)
    print("hall %.0f x %.0f m: floor, markings, benches, rack, robot cell, dock, %d pillars, 2 loaded bays"
          % (HALL_X, HALL_Y, len(PILLARS)))
    print("agents in the world: person_standing, ped_2, ped_3, cart (moved by tools/sim_agents.py)")
    print("world updated:", WORLD)
    print("next: colcon build --symlink-install, check Gazebo, then remap")


if __name__ == "__main__":
    main()