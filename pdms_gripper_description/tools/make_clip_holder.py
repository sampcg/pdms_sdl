#!/usr/bin/env python3
"""Generate a horizontal-extraction pipette holder as a COLLADA mesh.

This is the design worked out earlier in analysis: instead of dropping the
syringe down a shallow vertical bore, an open-fronted C-clip lets the arm pull it
out horizontally, along the same axis it approaches on.

Nothing here is drawn by hand - every dimension is a named constant below, so the
part can be re-derived if the syringe or the arm changes.

Local frame (matches the existing holder so it drops into the same joint):
    origin  centre of the mounting face, midway between the two bolts
    +X      along the bolt line, bolts at x = +-27
    +Y      away from the mounting face, towards the robot; flange y = 0..10
    +Z      up, parallel to the syringe axis; z = 0 is the bolt centreline

Writes meshes/pipette_holder_cantilever_clip.dae. The original
pipette_holder_2020.dae is left untouched.
"""
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
OUT = os.path.join(PKG, "meshes", "pipette_holder_cantilever_clip.dae")

# ---- flange: must stay 70 wide to carry bolts at +-27 ----------------------
FL_HW, FL_T = 35.0, 10.0
BAND_HW = 20.0              # everything lives in z = -20 .. +20
BOLT_X, BOLT_R = 27.0, 2.75

# ---- cantilever arm: narrowed to 45 so stations can sit at 110 mm pitch ----
ARM_HW = 22.5
CLIP_Y = 133.0              # clip centre, same as the existing holder's bore

# ---- clip -----------------------------------------------------------------
CLIP_R = 9.25               # 18.5 dia: syringe is 14.5-18 mm through this band
CLIP_WALL = 5.0
SLOT_W = 14.0               # < 18.5 so the clip retains the syringe
FLARE_W = 24.0              # lead-in mouth
FLARE_D = 5.0
KEY_D = 2.0                 # anti-rotation flat depth
CLIP_Z0, CLIP_Z1 = -20.0, 20.0      # 40 mm contact length (was 8 mm)

# ---- weight-bearing shelf under the syringe shoulder -----------------------
SHELF_Z0, SHELF_Z1 = -24.0, -20.0
SHELF_HOLE_R = 7.5          # 15 dia: passes the 14.5 shaft, catches the 18 shoulder
SHELF_SLOT_W = 15.5

TRIS = []


def quad(a, b, c, d):
    TRIS.append((a, b, c))
    TRIS.append((a, c, d))


def box(x0, x1, y0, y1, z0, z1):
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quad(p[0], p[3], p[2], p[1])          # bottom
    quad(p[4], p[5], p[6], p[7])          # top
    quad(p[0], p[1], p[5], p[4])          # -Y
    quad(p[2], p[3], p[7], p[6])          # +Y
    quad(p[1], p[2], p[6], p[5])          # +X
    quad(p[3], p[0], p[4], p[7])          # -X


def ring(cx, cy, r, n=32, a0=0.0, a1=2 * math.pi):
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
             cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def hole_wall(cx, cy, r, y0, y1, n=24):
    """Cylindrical bore with its axis along +Y (for the bolt holes)."""
    for i in range(n):
        a, b = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = (cx + r * math.cos(a), y0, cy + r * math.sin(a))
        p1 = (cx + r * math.cos(b), y0, cy + r * math.sin(b))
        p2 = (cx + r * math.cos(b), y1, cy + r * math.sin(b))
        p3 = (cx + r * math.cos(a), y1, cy + r * math.sin(a))
        quad(p0, p1, p2, p3)


def face_with_hole(cx, cz, r, y, half, n=24, flip=False):
    """Annulus from a circle out to a `half` x `half` square, on a plane y=const.

    Avoids needing a general polygon triangulator: the square patch around each
    bolt hole is meshed as a strip, and the rest of the flange face is filled
    with plain rectangles by the caller.
    """
    for i in range(n):
        a, b = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        ci = (cx + r * math.cos(a), y, cz + r * math.sin(a))
        cj = (cx + r * math.cos(b), y, cz + r * math.sin(b))
        # matching point on the square perimeter, same parametric position
        def sq(t):
            t = (t % (2 * math.pi)) / (2 * math.pi) * 4
            k, f = int(t), t - int(t)
            pts = [(1, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)]
            x0, z0 = pts[k]
            x1, z1 = pts[k + 1]
            return (cx + half * (x0 + (x1 - x0) * f), y, cz + half * (z0 + (z1 - z0) * f))
        si, sj = sq(a), sq(b)
        if flip:
            quad(ci, si, sj, cj)
        else:
            quad(ci, cj, sj, si)


def clip_profile():
    """C-shaped cross-section: bore, wall, slot opening towards +Y, and a flat."""
    inner, outer = [], []
    # the slot subtends this half-angle about +Y
    th = math.asin(min(1.0, (SLOT_W / 2) / CLIP_R))
    a0, a1 = math.pi / 2 + th, 2 * math.pi + math.pi / 2 - th
    for x, y in ring(0.0, CLIP_Y, CLIP_R, 40, a0, a1):
        # anti-rotation flat on the -Y side of the bore
        if y < CLIP_Y - (CLIP_R - KEY_D):
            y = CLIP_Y - (CLIP_R - KEY_D)
        inner.append((x, y))
    for x, y in ring(0.0, CLIP_Y, CLIP_R + CLIP_WALL, 40, a0, a1):
        outer.append((x, y))
    return inner, outer


def extrude_strip(inner, outer, z0, z1):
    for i in range(len(inner) - 1):
        i0, i1 = inner[i], inner[i + 1]
        o0, o1 = outer[i], outer[i + 1]
        quad((i0[0], i0[1], z0), (i1[0], i1[1], z0), (o1[0], o1[1], z0), (o0[0], o0[1], z0))
        quad((i0[0], i0[1], z1), (o0[0], o0[1], z1), (o1[0], o1[1], z1), (i1[0], i1[1], z1))
        quad((i0[0], i0[1], z0), (o0[0], o0[1], z0), (o0[0], o0[1], z1), (i0[0], i0[1], z1))
        quad((i1[0], i1[1], z0), (i1[0], i1[1], z1), (o1[0], o1[1], z1), (o1[0], o1[1], z0))
    for seq in (inner, outer):
        for i in range(len(seq) - 1):
            a, b = seq[i], seq[i + 1]
            quad((a[0], a[1], z0), (a[0], a[1], z1), (b[0], b[1], z1), (b[0], b[1], z0))


def build():
    # --- flange, with a meshed patch around each bolt hole ---
    for sx in (-1, 1):
        cx = sx * BOLT_X
        hole_wall(cx, 0.0, BOLT_R, 0.0, FL_T)
        face_with_hole(cx, 0.0, BOLT_R, 0.0, 10.0, flip=True)
        face_with_hole(cx, 0.0, BOLT_R, FL_T, 10.0)
    # remaining flange faces as rectangles, skipping the two 20x20 patches
    for y, fl in ((0.0, True), (FL_T, False)):
        quad((-FL_HW, y, -BAND_HW), (FL_HW, y, -BAND_HW), (FL_HW, y, -10.0), (-FL_HW, y, -10.0))
        quad((-FL_HW, y, 10.0), (FL_HW, y, 10.0), (FL_HW, y, BAND_HW), (-FL_HW, y, BAND_HW))
        quad((-FL_HW, y, -10.0), (-BOLT_X - 10, y, -10.0), (-BOLT_X - 10, y, 10.0), (-FL_HW, y, 10.0))
        quad((-BOLT_X + 10, y, -10.0), (BOLT_X - 10, y, -10.0), (BOLT_X - 10, y, 10.0), (-BOLT_X + 10, y, 10.0))
        quad((BOLT_X + 10, y, -10.0), (FL_HW, y, -10.0), (FL_HW, y, 10.0), (BOLT_X + 10, y, 10.0))
    # flange rim
    quad((-FL_HW, 0, -BAND_HW), (-FL_HW, FL_T, -BAND_HW), (FL_HW, FL_T, -BAND_HW), (FL_HW, 0, -BAND_HW))
    quad((-FL_HW, 0, BAND_HW), (FL_HW, 0, BAND_HW), (FL_HW, FL_T, BAND_HW), (-FL_HW, FL_T, BAND_HW))
    quad((-FL_HW, 0, -BAND_HW), (-FL_HW, 0, BAND_HW), (-FL_HW, FL_T, BAND_HW), (-FL_HW, FL_T, -BAND_HW))
    quad((FL_HW, 0, -BAND_HW), (FL_HW, FL_T, -BAND_HW), (FL_HW, FL_T, BAND_HW), (FL_HW, 0, BAND_HW))

    # --- cantilever arm ---
    box(-ARM_HW, ARM_HW, FL_T, CLIP_Y - CLIP_R - CLIP_WALL + 2.0, -BAND_HW, BAND_HW)

    # --- clip body ---
    inner, outer = clip_profile()
    extrude_strip(inner, outer, CLIP_Z0, CLIP_Z1)

    # --- lead-in flare: a short tapered mouth on the +Y opening ---
    for z0, z1 in ((CLIP_Z0, CLIP_Z0 + 4.0), (CLIP_Z1 - 4.0, CLIP_Z1)):
        pass  # flare is in-plane, added below as angled jaw faces
    yj = CLIP_Y + CLIP_R * math.cos(math.asin(min(1.0, (SLOT_W / 2) / CLIP_R)))
    for sx in (-1, 1):
        x_in, x_out = sx * SLOT_W / 2, sx * FLARE_W / 2
        quad((x_in, yj, CLIP_Z0), (x_out, yj + FLARE_D, CLIP_Z0),
             (x_out, yj + FLARE_D, CLIP_Z1), (x_in, yj, CLIP_Z1))

    # --- weight-bearing shelf ---
    n = 28
    th = math.asin(min(1.0, (SHELF_SLOT_W / 2) / SHELF_HOLE_R)) if SHELF_SLOT_W / 2 < SHELF_HOLE_R else 0.0
    a0, a1 = math.pi / 2 + th, 2 * math.pi + math.pi / 2 - th
    inner_s = ring(0.0, CLIP_Y, SHELF_HOLE_R, n, a0, a1)
    outer_s = ring(0.0, CLIP_Y, CLIP_R + CLIP_WALL, n, a0, a1)
    extrude_strip(inner_s, outer_s, SHELF_Z0, SHELF_Z1)


DAE = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><author>make_clip_holder.py</author></contributor>
    <unit meter="0.001" name="millimeter" />
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="clip_holder" name="clip_holder">
      <mesh>
        <source id="pos">
          <float_array id="pos-a" count="%d">%s</float_array>
          <technique_common>
            <accessor source="#pos-a" count="%d" stride="3">
              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="verts"><input semantic="POSITION" source="#pos"/></vertices>
        <triangles count="%d">
          <input semantic="VERTEX" source="#verts" offset="0"/>
          <p>%s</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="holder"><instance_geometry url="#clip_holder"/></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''


def write():
    verts, idx = [], []
    lut = {}
    for tri in TRIS:
        for p in tri:
            k = (round(p[0], 4), round(p[1], 4), round(p[2], 4))
            if k not in lut:
                lut[k] = len(verts)
                verts.append(k)
            idx.append(lut[k])
    flat = " ".join("%.4f" % v for p in verts for v in p)
    pstr = " ".join(str(i) for i in idx)
    open(OUT, "w").write(DAE % (len(verts) * 3, flat, len(verts), len(TRIS), pstr))
    V = np.array(verts)
    print("wrote", OUT)
    print("  %d verts, %d triangles" % (len(verts), len(TRIS)))
    print("  bbox (mm): %s .. %s" % (np.round(V.min(0), 2), np.round(V.max(0), 2)))
    print("  clip centre y=%.1f  contact length %.0f mm  bore dia %.1f"
          % (CLIP_Y, CLIP_Z1 - CLIP_Z0, CLIP_R * 2))
    tilt = math.degrees(math.atan2(CLIP_R * 2 - 16.0, CLIP_Z1 - CLIP_Z0))
    print("  max tilt with a 16 mm shaft: %.1f deg  (old 8 mm bore gave 14 deg)" % tilt)


if __name__ == "__main__":
    build()
    write()
