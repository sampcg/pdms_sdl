#!/usr/bin/env python3
"""Generate a 3-station mounting rail: mixer in the centre, a syringe each side.

Why a rail rather than three separate brackets:

  * the pipette holder's standoff (mount face -> bore) is 133.0 mm, but the mixer
    holder's is only 24.4 mm. Bolted to the same bar face the mixer would sit
    108.6 mm further from the robot - at 518 mm, past the 472 mm reach. The rail
    carries the mixer on a boss that makes up exactly that difference, so all
    three stations land on one arc.
  * three independent brackets each deflect differently under the extraction
    pull, and their pitch tolerances stack.
  * one backplate can take 6 M5 bolts in the T-slot instead of 2, which matters
    because horizontal extraction loads the mount in bending.

Local frame (same convention as the other holders):
    origin  centre of the mounting face, on the bolt centreline
    +X      along the bolt line / along the crossbar
    +Y      away from the mounting face, towards the robot
    +Z      up; z = 0 is the bolt centreline

Writes meshes/station_rail.dae
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
OUT = os.path.join(PKG, "meshes", "station_rail.dae")

PITCH = 115.0            # >= 115 mm: 140 mm gripper envelope at transit width
PLATE_T = 8.0            # backplate thickness
BAND = 22.0              # half-height of the rail, z = -22 .. +22
PAD_HW = 32.0            # each mounting pad is 64 wide, to carry bolts at +-27
BOLT_X, BOLT_R = 27.0, 2.75

PAD_T = 8.0              # thickness of the pad the mixer holder bolts to
# The boss brings the mixer's 24.4 mm standoff up to the pipettes' 133 mm, so
# every station's axis lands at the same distance from the bar face:
#     PLATE_T + BOSS + PAD_T + 24.4  ==  PLATE_T + 133
MIXER_BOSS = 133.0 - 24.4 - PAD_T     # 100.6 mm

TRIS = []


def quad(a, b, c, d):
    TRIS.append((a, b, c)); TRIS.append((a, c, d))


def box(x0, x1, y0, y1, z0, z1):
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quad(p[0], p[3], p[2], p[1]); quad(p[4], p[5], p[6], p[7])
    quad(p[0], p[1], p[5], p[4]); quad(p[2], p[3], p[7], p[6])
    quad(p[1], p[2], p[6], p[5]); quad(p[3], p[0], p[4], p[7])


def bolt_hole(cx, cz, y0, y1, n=24):
    for i in range(n):
        a, b = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = (cx + BOLT_R * math.cos(a), y0, cz + BOLT_R * math.sin(a))
        p1 = (cx + BOLT_R * math.cos(b), y0, cz + BOLT_R * math.sin(b))
        p2 = (cx + BOLT_R * math.cos(b), y1, cz + BOLT_R * math.sin(b))
        p3 = (cx + BOLT_R * math.cos(a), y1, cz + BOLT_R * math.sin(a))
        quad(p0, p1, p2, p3)


def build():
    half = PITCH + PAD_HW
    # backplate spanning all three stations
    box(-half, half, 0.0, PLATE_T, -BAND, BAND)
    # six M5 holes: a pair at each station, 54 mm apart along the bar
    for xc in (-PITCH, 0.0, PITCH):
        for sx in (-1, 1):
            bolt_hole(xc + sx * BOLT_X, 0.0, 0.0, PLATE_T)
    # mixer boss: brings its 24.4 mm standoff up to the pipettes' 133 mm
    box(-PAD_HW, PAD_HW, PLATE_T, PLATE_T + MIXER_BOSS, -BAND, BAND)
    # a pad on the front of the boss for the mixer holder to bolt to
    for sx in (-1, 1):
        bolt_hole(sx * BOLT_X, 0.0, PLATE_T + MIXER_BOSS, PLATE_T + MIXER_BOSS + PAD_T)
    box(-PAD_HW, PAD_HW, PLATE_T + MIXER_BOSS, PLATE_T + MIXER_BOSS + PAD_T, -BAND, BAND)
    # gussets either side of the boss, so the extraction pull does not bend it
    for sx in (-1, 1):
        quad((sx * PAD_HW, PLATE_T, -BAND), (sx * PAD_HW, PLATE_T + MIXER_BOSS, -BAND),
             (sx * PAD_HW, PLATE_T + MIXER_BOSS, BAND), (sx * PAD_HW, PLATE_T, BAND))


DAE = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><author>make_station_rail.py</author></contributor>
    <unit meter="0.001" name="millimeter" /><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="rail" name="rail"><mesh>
    <source id="pos"><float_array id="pos-a" count="%d">%s</float_array>
      <technique_common><accessor source="#pos-a" count="%d" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>
    <triangles count="%d"><input semantic="VERTEX" source="#verts" offset="0"/><p>%s</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene">
    <node id="rail"><instance_geometry url="#rail"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''


def write():
    verts, idx, lut = [], [], {}
    for tri in TRIS:
        for p in tri:
            k = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
            if k not in lut:
                lut[k] = len(verts); verts.append(k)
            idx.append(lut[k])
    open(OUT, "w").write(DAE % (len(verts) * 3,
                                " ".join("%.4f" % v for p in verts for v in p),
                                len(verts), len(TRIS),
                                " ".join(str(i) for i in idx)))
    V = np.array(verts)
    print("wrote", OUT)
    print("  %d verts, %d tris" % (len(verts), len(TRIS)))
    print("  bbox (mm): %s .. %s" % (np.round(V.min(0), 1), np.round(V.max(0), 1)))
    print("  stations at x = %+.0f (syringe), 0 (mixer), %+.0f (syringe)" % (-PITCH, PITCH))
    print("  mixer boss %.1f mm; all stations at %.1f mm from the bar face"
          % (MIXER_BOSS, PLATE_T + 133.0))
    print("  6 x M5 at x = %s" % ", ".join(
        "%+.0f" % (xc + sx * BOLT_X) for xc in (-PITCH, 0.0, PITCH) for sx in (-1, 1)))


if __name__ == "__main__":
    build()
    write()
