#!/usr/bin/env python3
"""
make_warehouse_world.py -- turn the test room into a small, realistic warehouse
that gives the camera plenty to track (for visual odometry) and still looks like a real place.

What it does
  1. Generates textures in src/my_robot/worlds/textures/:
       floor_concrete.png      concrete tiles, seams, stains, yellow safety lanes, a few floor markings
       wall_panel_<dir>.png    light grey wall panels, hazard kick-plate, safety signs and posters (one per wall)
       box_a/b/c.png           cardboard: tape, shipping labels, barcodes, handling marks
       pallet_wood.png         wooden pallet planks
     and removes the old random-color textures (floor.png, wall_*.png) if present.
  2. Patches src/my_robot/worlds/my_world.sdf:
       - walls get the panel texture (wall geometry unchanged)
       - 'vio_floor' (visual only, 2 mm, no collision) gets the concrete texture
       - 9 'prop_*' models: pallets with stacked boxes and loose boxes, along the walls only;
         the centre of the room and the person's walking lane stay free
     Backup on first run: my_world.sdf.prewarehouse.bak. Safe to run again (replaces its own models).

After running: colcon build --symlink-install, look at it in Gazebo, then REMAP
(the LiDAR now sees the props, so the old saved map no longer matches).

If the wall signs show upside down or mirrored in Gazebo, set FLIP_V / FLIP_H below and run again.

Run:  python3 ~/autonomy/tools/make_warehouse_world.py
"""
import math
import os
import re
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

FLIP_V = False      # set True if wall signs appear upside down in Gazebo
FLIP_H = False      # set True if wall text appears mirrored in Gazebo

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_DIR = os.path.join(ROOT, "src", "my_robot", "worlds")
WORLD = os.path.join(WORLD_DIR, "my_world.sdf")
TEX = os.path.join(WORLD_DIR, "textures")
WALLS = {"wall_north": "north", "wall_south": "south", "wall_east": "east", "wall_west": "west"}
OLD_CONFETTI = ["floor.png", "wall_north.png", "wall_south.png", "wall_east.png", "wall_west.png"]

# Props along the walls: (name, x, y, yaw, kind). Room interior is -5.95..5.95 m.
# Person walks y=3.5, x in [1.5, 5.5]; robot starts at (0, 0). Both areas stay clear.
PROPS = [
    ("prop_n1", -4.0,  5.3, 0.0,          "pallet"),
    ("prop_n2", -1.2,  5.3, 0.0,          "pallet"),
    ("prop_n3",  1.8,  5.35, 0.0,         "loose"),
    ("prop_s1", -3.8, -5.3, 0.0,          "pallet"),
    ("prop_s2", -0.6, -5.3, 0.0,          "loose"),
    ("prop_s3",  3.6, -5.3, 0.0,          "pallet"),
    ("prop_w1", -5.3, -2.2, math.pi / 2,  "pallet"),
    ("prop_w2", -5.3,  1.6, math.pi / 2,  "pallet"),
    ("prop_e1",  5.3, -2.6, math.pi / 2,  "pallet"),
]


# ----------------------------------------------------------------------------- helpers
def font(size, bold=True):
    names = ["DejaVuSans-Bold.ttf", "DejaVuSans.ttf"] if bold else ["DejaVuSans.ttf"]
    for n in names:
        for d in ["/usr/share/fonts/truetype/dejavu", ""]:
            try:
                return ImageFont.truetype(os.path.join(d, n) if d else n, size)
            except OSError:
                pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def blotches(w, h, rng, cells, amp):
    """Smooth low-frequency noise in [-amp, amp]."""
    small = rng.uniform(-amp, amp, (cells[1], cells[0])).astype(np.float32)
    im = Image.fromarray(small, mode="F").resize((w, h), Image.BICUBIC)
    return np.asarray(im)


def to_img(arr):
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def text_center(d, box, txt, size, fill):
    f = font(size)
    l, t, r, b = d.textbbox((0, 0), txt, font=f)
    x0, y0, x1, y1 = box
    d.text((x0 + (x1 - x0 - (r - l)) / 2 - l, y0 + (y1 - y0 - (b - t)) / 2 - t), txt, font=f, fill=fill)


def barcode(d, x, y, w, h, rng):
    cx = x
    while cx < x + w - 3:
        bw = int(rng.integers(1, 4))
        if rng.random() < 0.55:
            d.rectangle([cx, y, cx + bw - 1, y + h], fill=(20, 20, 20))
        cx += bw + int(rng.integers(1, 3))


# ----------------------------------------------------------------------------- textures
def floor_concrete(size=2048, room=11.9):
    rng = np.random.default_rng(7)
    ppm = size / room
    base = 150 + blotches(size, size, rng, (24, 24), 16) + blotches(size, size, rng, (180, 180), 6)
    base += rng.normal(0, 4, (size, size))
    rgb = np.stack([base, base * 0.99, base * 0.96], axis=-1)
    img = to_img(rgb)
    d = ImageDraw.Draw(img)

    # stains
    stain = Image.new("L", (size, size), 0)
    sd = ImageDraw.Draw(stain)
    for _ in range(40):
        cx, cy = rng.integers(0, size, 2)
        rx, ry = rng.integers(10, 70, 2)
        sd.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=int(rng.integers(25, 70)))
    stain = stain.filter(ImageFilter.GaussianBlur(9))
    img = Image.composite(Image.new("RGB", (size, size), (85, 82, 78)), img, stain)
    d = ImageDraw.Draw(img)

    # tile seams every 1 m
    k = 1
    while k * ppm < size:
        p = int(k * ppm)
        d.line([(p, 0), (p, size)], fill=(98, 96, 92), width=2)
        d.line([(0, p), (size, p)], fill=(98, 96, 92), width=2)
        k += 1

    # yellow safety lane loop 1.7 m in from the walls, slightly worn
    yellow = (226, 184, 32)
    inset = int(1.7 * ppm)
    lw = int(0.07 * ppm)
    for (x0, y0, x1, y1) in [(inset, inset, size - inset, inset + lw), (inset, size - inset - lw, size - inset, size - inset),
                             (inset, inset, inset + lw, size - inset), (size - inset - lw, inset, size - inset, size - inset)]:
        d.rectangle([x0, y0, x1, y1], fill=yellow)
    for _ in range(260):  # wear
        x, y = rng.integers(0, size, 2)
        r = int(rng.integers(2, 7))
        d.ellipse([x - r, y - r, x + r, y + r], fill=tuple(int(v) for v in (150 + rng.integers(-10, 10, 3))))

    # floor markings: a few white chevrons and zone numbers in the open area
    for i, (mx, my) in enumerate([(0.35, 0.5), (0.65, 0.5), (0.5, 0.33), (0.5, 0.67)]):
        cx, cy = int(mx * size), int(my * size)
        s = int(0.25 * ppm)
        d.polygon([(cx - s, cy - s // 2), (cx, cy + s // 2), (cx + s, cy - s // 2), (cx + s, cy), (cx, cy + s), (cx - s, cy)],
                  fill=(232, 232, 228))
        text_center(d, (cx - 2 * s, cy + s + 6, cx + 2 * s, cy + s + 6 + int(0.22 * ppm)), "Z%d" % (i + 1),
                    int(0.18 * ppm), (232, 232, 228))
    return img


SIGNS = [
    ("warn", "CAUTION", "FORKLIFTS"),
    ("exit", "EXIT", ""),
    ("fire", "FIRE", "EXTINGUISHER"),
    ("aisle", "AISLE", "B3"),
    ("aisle", "ZONE", "2"),
    ("poster", "SAFETY FIRST", ""),
    ("warn", "WATCH", "YOUR STEP"),
    ("poster", "DAILY CHECK", ""),
    ("aisle", "DOCK", "4"),
    ("ppe", "PPE", "REQUIRED"),
]


def draw_sign(img, kind, t1, t2, x, y, w, h, rng):
    d = ImageDraw.Draw(img)
    if kind == "warn":
        d.rectangle([x, y, x + w, y + h], fill=(250, 210, 30), outline=(20, 20, 20), width=4)
        tri = [(x + w / 2, y + 12), (x + w / 2 - h * 0.3, y + h * 0.58), (x + w / 2 + h * 0.3, y + h * 0.58)]
        d.polygon(tri, fill=(20, 20, 20))
        text_center(d, (x, y + 20, x + w, y + h * 0.58), "!", int(h * 0.3), (250, 210, 30))
        text_center(d, (x + 4, y + h * 0.6, x + w - 4, y + h * 0.78), t1, int(h * 0.13), (20, 20, 20))
        text_center(d, (x + 4, y + h * 0.78, x + w - 4, y + h * 0.95), t2, int(h * 0.1), (20, 20, 20))
    elif kind == "exit":
        d.rectangle([x, y, x + w, y + h], fill=(18, 140, 70))
        text_center(d, (x, y, x + w * 0.65, y + h), t1, int(h * 0.45), (255, 255, 255))
        ax = x + w * 0.7
        d.polygon([(ax, y + h * 0.4), (ax + w * 0.15, y + h * 0.4), (ax + w * 0.15, y + h * 0.25), (ax + w * 0.27, y + h * 0.5),
                   (ax + w * 0.15, y + h * 0.75), (ax + w * 0.15, y + h * 0.6), (ax, y + h * 0.6)], fill=(255, 255, 255))
    elif kind == "fire":
        d.rectangle([x, y, x + w, y + h], fill=(200, 30, 35))
        cx = x + w / 2
        d.rounded_rectangle([cx - w * 0.12, y + h * 0.18, cx + w * 0.12, y + h * 0.62], 8, fill=(255, 255, 255))
        d.rectangle([cx - w * 0.04, y + h * 0.08, cx + w * 0.04, y + h * 0.18], fill=(255, 255, 255))
        text_center(d, (x, y + h * 0.64, x + w, y + h * 0.8), t1, int(h * 0.13), (255, 255, 255))
        text_center(d, (x, y + h * 0.8, x + w, y + h * 0.95), t2, int(h * 0.09), (255, 255, 255))
    elif kind == "aisle":
        d.rectangle([x, y, x + w, y + h], fill=(25, 70, 160), outline=(255, 255, 255), width=5)
        text_center(d, (x, y + h * 0.06, x + w, y + h * 0.4), t1, int(h * 0.22), (255, 255, 255))
        text_center(d, (x, y + h * 0.38, x + w, y + h * 0.95), t2, int(h * 0.48), (255, 255, 255))
    elif kind == "ppe":
        d.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(20, 70, 170), width=4)
        d.ellipse([x + w * 0.3, y + h * 0.1, x + w * 0.7, y + h * 0.1 + w * 0.4], fill=(20, 70, 170))
        d.rectangle([x + w * 0.38, y + h * 0.1 + w * 0.12, x + w * 0.62, y + h * 0.1 + w * 0.22], fill=(255, 255, 255))
        text_center(d, (x, y + h * 0.62, x + w, y + h * 0.78), t1, int(h * 0.13), (20, 70, 170))
        text_center(d, (x, y + h * 0.78, x + w, y + h * 0.94), t2, int(h * 0.09), (20, 70, 170))
    else:  # poster: white sheet, coloured header, text lines and a small bar chart
        d.rectangle([x, y, x + w, y + h], fill=(246, 246, 240), outline=(120, 120, 120), width=2)
        hc = tuple(int(v) for v in rng.choice([(200, 60, 40), (30, 110, 170), (40, 140, 90)]))
        d.rectangle([x, y, x + w, y + h * 0.18], fill=hc)
        text_center(d, (x, y, x + w, y + h * 0.18), t1, int(h * 0.09), (255, 255, 255))
        for i in range(5):
            ly = y + h * (0.26 + i * 0.08)
            d.rectangle([x + w * 0.08, ly, x + w * (0.5 + 0.4 * rng.random()), ly + 4], fill=(90, 90, 90))
        for i in range(5):
            bh = h * (0.1 + 0.18 * rng.random())
            bx = x + w * (0.12 + i * 0.16)
            d.rectangle([bx, y + h * 0.9 - bh, bx + w * 0.1, y + h * 0.9], fill=hc)


def wall_panel(seed, w=4096, h=342):
    rng = np.random.default_rng(seed)
    ppm = w / 12.0
    base = 205 + blotches(w, h, rng, (60, 6), 7) + rng.normal(0, 2.5, (h, w))
    img = to_img(np.stack([base, base, base * 1.01], axis=-1))
    d = ImageDraw.Draw(img)
    # panel seams every 1.2 m
    k = 1
    while k * 1.2 * ppm < w:
        p = int(k * 1.2 * ppm)
        d.line([(p, 0), (p, h)], fill=(150, 150, 152), width=3)
        d.line([(p + 3, 0), (p + 3, h)], fill=(225, 225, 228), width=1)
        k += 1
    # darker band 0.12-0.28 m and hazard kick plate 0-0.12 m (image y grows downward = wall top to bottom)
    band_top, kick_top = int(h * (1 - 0.28)), int(h * (1 - 0.12))
    d.rectangle([0, band_top, w, kick_top], fill=(110, 112, 116))
    stripe = int(0.1 * ppm)
    for sx in range(-h, w + h, stripe * 2):
        d.polygon([(sx, h), (sx + stripe, h), (sx + stripe + (h - kick_top), kick_top), (sx + (h - kick_top), kick_top)],
                  fill=(240, 196, 25))
    d.rectangle([0, kick_top, w, h], outline=None)
    hz = Image.new("RGB", (w, h - kick_top), (25, 25, 25))
    hd = ImageDraw.Draw(hz)
    for sx in range(-60, w + 60, stripe * 2):
        hd.polygon([(sx, h - kick_top), (sx + stripe, h - kick_top), (sx + stripe + 40, 0), (sx + 40, 0)], fill=(240, 196, 25))
    img.paste(hz, (0, kick_top))
    # signs and posters at 0.35-0.95 m height, no overlap
    order = rng.permutation(len(SIGNS))
    placed, n = [], 0
    for idx in order:
        if n >= 7:
            break
        kind, t1, t2 = SIGNS[idx]
        if kind == "aisle":  # unique numbers per wall, so no two walls look the same
            t2 = {"AISLE": "%s%d" % ("ABCDEFGH"[seed % 8], int(rng.integers(1, 10))),
                  "ZONE": str(int(rng.integers(1, 10))), "DOCK": str(int(rng.integers(1, 13)))}.get(t1, t2)
        sw = int((0.62 if kind in ("exit", "aisle") else 0.45) * ppm)
        sh = int((0.3 if kind == "exit" else 0.55) * ppm)
        for _ in range(60):
            sx = int(rng.integers(30, w - sw - 30))
            sy = int(h * (1 - 0.95) + rng.integers(0, max(1, int(h * 0.6) - sh)))
            if all(sx + sw + 60 < px or sx > px + pw + 60 for px, pw in placed):
                draw_sign(img, kind, t1, t2, sx, sy, sw, sh, rng)
                placed.append((sx, sw))
                n += 1
                break
    if FLIP_V:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
    if FLIP_H:
        img = img.transpose(Image.FLIP_LEFT_RIGHT)
    return img


def cardboard(seed, size=512):
    rng = np.random.default_rng(seed)
    base = 1.0 + blotches(size, size, rng, (12, 12), 0.06) + rng.normal(0, 0.025, (size, size))
    fib = blotches(size, size, rng, (size // 2, 8), 0.05)  # vertical fibres
    tone = np.array([178, 136, 88], dtype=np.float32)
    rgb = (base + fib)[..., None] * tone
    img = to_img(rgb)
    d = ImageDraw.Draw(img)
    # packing tape across the middle
    tape = Image.new("RGBA", (size, 70), (205, 170, 115, 150))
    img.paste(tape, (0, size // 2 - 35), tape)
    d = ImageDraw.Draw(img)
    d.line([(0, size // 2 - 35), (size, size // 2 - 35)], fill=(150, 112, 70), width=2)
    d.line([(0, size // 2 + 35), (size, size // 2 + 35)], fill=(150, 112, 70), width=2)
    # shipping label with text lines and barcode
    lx, ly = int(rng.integers(40, 200)), int(rng.integers(300, 360))
    d.rectangle([lx, ly, lx + 220, ly + 130], fill=(250, 250, 248), outline=(60, 60, 60), width=2)
    d.text((lx + 10, ly + 8), "SHIP TO: DC-%02d" % rng.integers(1, 40), font=font(18), fill=(20, 20, 20))
    for i in range(3):
        d.rectangle([lx + 10, ly + 36 + i * 14, lx + 10 + int(rng.integers(90, 190)), ly + 42 + i * 14], fill=(70, 70, 70))
    barcode(d, lx + 10, ly + 82, 200, 38, rng)
    # handling marks
    v = seed % 3
    if v == 0:
        text_center(d, (20, 20, size - 20, 120), "FRAGILE", 64, (180, 30, 30))
        d.rectangle([20, 20, size - 20, 120], outline=(180, 30, 30), width=5)
    elif v == 1:
        for ax in (size * 0.3, size * 0.55):
            d.polygon([(ax, 40), (ax + 40, 100), (ax + 15, 100), (ax + 15, 150), (ax - 15, 150), (ax - 15, 100), (ax - 40, 100)],
                      fill=(30, 30, 30))
        text_center(d, (0, 160, size, 200), "THIS SIDE UP", 26, (30, 30, 30))
    else:
        d.text((30, 40), "LOT %04d" % rng.integers(1000, 9999), font=font(54), fill=(40, 40, 40))
        d.text((30, 110), "QTY 24", font=font(40), fill=(40, 40, 40))
    # darker edges
    edge = Image.new("L", (size, size), 0)
    ed = ImageDraw.Draw(edge)
    ed.rectangle([0, 0, size - 1, size - 1], outline=110, width=18)
    edge = edge.filter(ImageFilter.GaussianBlur(10))
    img = Image.composite(Image.new("RGB", (size, size), (110, 80, 50)), img, edge)
    return img


def pallet_wood(size=512):
    rng = np.random.default_rng(11)
    y = np.arange(size, dtype=np.float32)[:, None]
    x = np.arange(size, dtype=np.float32)[None, :]
    grain = 0.07 * np.sin(y * 0.35 + 6 * blotches(size, size, rng, (6, 20), 1.0)) + blotches(size, size, rng, (40, 4), 0.06)
    rgb = (1.0 + grain + rng.normal(0, 0.02, (size, size)))[..., None] * np.array([196, 160, 112], dtype=np.float32)
    img = to_img(rgb)
    d = ImageDraw.Draw(img)
    for gy in range(0, size, 102):  # gaps between planks
        d.rectangle([0, gy, size, gy + 6], fill=(70, 52, 34))
    for _ in range(14):  # nails
        nx, ny = rng.integers(10, size - 10, 2)
        d.ellipse([nx - 3, ny - 3, nx + 3, ny + 3], fill=(60, 60, 60))
    return img


# ----------------------------------------------------------------------------- SDF
def mat(tex):
    return ("<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>"
            "<pbr><metal><albedo_map>textures/%s</albedo_map><roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr>"
            "</material>" % tex)


def box_elems(name, x, y, z, yaw, sx, sy, sz, tex, collide=True):
    pose = "%.3f %.3f %.3f 0 0 %.3f" % (x, y, z, yaw)
    geo = "<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>" % (sx, sy, sz)
    s = '        <visual name="v_%s"><pose>%s</pose>%s%s</visual>\n' % (name, pose, geo, mat(tex))
    if collide:
        s += '        <collision name="c_%s"><pose>%s</pose>%s</collision>\n' % (name, pose, geo)
    return s


BOX_SIZES = [(0.55, 0.38, 0.36), (0.42, 0.42, 0.40), (0.58, 0.40, 0.30), (0.36, 0.30, 0.26), (0.50, 0.36, 0.44)]


def prop_model(name, px, py, yaw, kind, rng):
    body = ""
    if kind == "pallet":
        # deck + three runners, one collision block for the whole pallet
        body += box_elems("deck", 0, 0, 0.129, 0, 1.2, 0.8, 0.03, "pallet_wood.png", collide=False)
        for i, yy in enumerate((-0.35, 0.0, 0.35)):
            body += box_elems("run%d" % i, 0, yy, 0.057, 0, 1.2, 0.1, 0.114, "pallet_wood.png", collide=False)
        body += '        <collision name="c_pallet"><pose>0 0 0.072 0 0 0</pose><geometry><box><size>1.2 0.8 0.144</size></box></geometry></collision>\n'
        z0, slots = 0.144, [(-0.3, 0.0), (0.3, 0.0)]
    else:
        z0, slots = 0.0, [(-0.35, 0.05), (0.25, -0.05), (0.75, 0.08)]
    k = 0
    for (sx, sy) in slots:
        z = z0
        for _ in range(int(rng.integers(1, 4 if kind == "pallet" else 3))):
            w, dpt, h = BOX_SIZES[int(rng.integers(0, len(BOX_SIZES)))]
            jx, jy = rng.uniform(-0.02, 0.02, 2)
            byaw = rng.uniform(-0.08, 0.08)
            tex = "box_%s.png" % "abc"[int(rng.integers(0, 3))]
            body += box_elems("box%d" % k, sx + jx, sy + jy, z + h / 2, byaw, w, dpt, h, tex)
            z += h
            k += 1
    return ('    <model name="%s">\n      <static>true</static>\n      <pose>%.3f %.3f 0 0 0 %.4f</pose>\n'
            '      <link name="link">\n%s      </link>\n    </model>\n' % (name, px, py, yaw, body))


FLOOR_MODEL = ('    <model name="vio_floor">\n      <static>true</static>\n      <pose>0 0 0.001 0 0 0</pose>\n'
               '      <link name="link">\n        <visual name="v"><geometry><box><size>11.9 11.9 0.002</size></box></geometry>%s</visual>\n'
               '      </link>\n    </model>\n') % mat("floor_concrete.png")


def remove_model(sdf, name_regex):
    return re.sub(r'\n?[ \t]*<model name="%s">.*?\n[ \t]*</model>[ \t]*' % name_regex, "", sdf, flags=re.S)


def main():
    if not os.path.isfile(WORLD):
        raise SystemExit("World file not found: " + WORLD)
    os.makedirs(TEX, exist_ok=True)

    floor_concrete().save(os.path.join(TEX, "floor_concrete.png"))
    for i, (wall, d) in enumerate(WALLS.items()):
        wall_panel(seed=300 + i).save(os.path.join(TEX, "wall_panel_%s.png" % d))
    for i, v in enumerate("abc"):
        cardboard(seed=i).save(os.path.join(TEX, "box_%s.png" % v))
    pallet_wood().save(os.path.join(TEX, "pallet_wood.png"))
    for f in OLD_CONFETTI:
        p = os.path.join(TEX, f)
        if os.path.exists(p):
            os.remove(p)
    print("textures written to", TEX)

    sdf = open(WORLD).read()
    backup = WORLD + ".prewarehouse.bak"
    if not os.path.exists(backup):
        shutil.copyfile(WORLD, backup)
        print("backup:", backup)

    for wall, d in WALLS.items():
        m = re.search(r'<model name="%s">.*?</model>' % wall, sdf, re.S)
        if not m:
            print("WARNING: %s not found, skipped" % wall)
            continue
        block = m.group(0)
        vis = re.search(r'<visual name="v">.*?</visual>', block, re.S)
        new_vis = re.sub(r'<material>.*?</material>', mat("wall_panel_%s.png" % d), vis.group(0), count=1, flags=re.S)
        sdf = sdf.replace(block, block.replace(vis.group(0), new_vis))
        print("%s: panel texture" % wall)

    sdf = remove_model(sdf, "vio_floor")
    sdf = remove_model(sdf, r"prop_[a-z0-9]+")
    rng = np.random.default_rng(42)
    models = FLOOR_MODEL + "".join(prop_model(n, x, y, yaw, kind, rng) for (n, x, y, yaw, kind) in PROPS)
    idx = sdf.rfind("</world>")
    sdf = sdf[:idx].rstrip() + "\n" + models + "  " + sdf[idx:]
    open(WORLD, "w").write(sdf)
    print("vio_floor: concrete; %d props added along the walls" % len(PROPS))
    print("world updated:", WORLD)
    print("next: colcon build --symlink-install, check Gazebo, then remap")


if __name__ == "__main__":
    main()