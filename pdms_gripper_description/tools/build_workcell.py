#!/usr/bin/env python3
"""Generate the workcell URDF: 320 arm + fork gripper + 2020 frame + pipette holder.

All placements are derived from mesh geometry, not eyeballed:

  frame       six 20x20 extrusions; lower Y crossbar spans y -519.72..-19.72
              at z 240..260, with vertical faces at x=-14.13 and x=+5.87.
  holder      8 mm flange at local y 0..8, two M5 bolt holes on a Y axis at
              local z=0, x=+-27. Bolting the flange face (y=0) onto the bar's
              +X face means local +y -> world +x, i.e. rpy = (0, 0, -pi/2).
  bore        r=10.0 through the holder shelf, axis Z, at local (x=0, y=35).
              After the -90 deg yaw that lands at frame-local (+35, 0) from the
              holder origin.

The frame is then positioned so the bore sits at BORE_IN_BASE in the robot's
base frame.

Run:  python3 build_workcell.py            (writes urdf/mycobot_320_pi_2022_workcell.urdf)
"""
import os
import re
import sys
import math

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.join(PKG, "urdf", "mycobot_320_pi_2022_fork_gripper.urdf")
STATIC_PIPETTE = "--static-pipette" in sys.argv
# --clip : use the generated horizontal-extraction holder and write a SEPARATE
# urdf, leaving the original workcell file and the original holder mesh alone.
CLIP = "--clip" in sys.argv
# --stations N : how many holder/syringe stations along the crossbar.
# Bores sit at +-PITCH/2 either side of the frame centreline, each holder yawed
# so its clip opening points at the robot base - otherwise horizontal extraction
# from an off-centre station pulls at an angle to the clip axis and binds.
try:
    N_STATIONS = int(sys.argv[sys.argv.index("--stations") + 1])
except (ValueError, IndexError):
    N_STATIONS = 1
STATION_PITCH = 0.110      # >= 102 mm: gripper envelope 140 mm + 45 mm body + clearance
HOLDER_MESH = "pipette_holder_cantilever_clip.dae" if CLIP else "pipette_holder_2020.dae"
SUFFIX = ("_clip" if CLIP else "") + ("_x%d" % N_STATIONS if N_STATIONS > 1 else "")
OUT = os.path.join(PKG, "urdf", "mycobot_320_pi_2022_workcell" + SUFFIX + ".urdf")
POSE_OUT = os.path.join(PKG, "config", "pipette_pose" + SUFFIX + ".yaml")

M = 0.001  # mesh units are mm

# --- derived geometry (mm) -------------------------------------------------
# The lower Y crossbar has vertical faces at x=-14.13 (-X) and x=+5.87 (+X).
# The robot stands on the -X side, so mount the holder on the -X face; its
# flange normal (local +y) must then point -X, i.e. yaw = +pi/2.
BAR_FACE_X = -14.13
BAR_CENTRE_Y = -269.72   # midpoint of that bar along its length
BAR_MID_Z = 180.0        # bar mid-height (frame revised 2026-07-29: was 250)
HOLDER_YAW = math.pi / 2
BORE_LOCAL = (0.0, 133.0)  # bore centre in holder x,y (cantilever holder)


def rotz(yaw, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


# bore position in frame coords, after the holder yaw
_b = rotz(HOLDER_YAW, (BORE_LOCAL[0], BORE_LOCAL[1], 0.0))
BORE_IN_FRAME = (BAR_FACE_X + _b[0], BAR_CENTRE_Y + _b[1], BAR_MID_Z + _b[2])

# where we want the bore to land relative to the robot base (metres).
# 300 mm keeps it inside the 320's ~350 mm reach while pushing the frame
# structure itself clear of the robot.
BORE_IN_BASE = (0.0, -0.410, 0.180)   # 410 mm: furthest standoff still reachable

# The arm's natural front is -Y (at q=0 the tool sits at y=-0.275), so yaw the
# frame -90 deg: the crossbar then runs along X and the holder's mounting face
# (-X in frame coords) turns to +Y, i.e. towards the robot.
FRAME_YAW = -math.pi / 2
_bf = rotz(FRAME_YAW, BORE_IN_FRAME)
FRAME_XYZ = tuple(BORE_IN_BASE[i] - _bf[i] * M for i in range(3))

# --- pipette ---------------------------------------------------------------
# shaft axis lies along mesh +Y at (x=-1.47, z=14.95); tip is the +Y end.
# rpy=(-pi/2,0,0) sends mesh +Y -> world -Z (tip down); then mesh (x,y,z)
# maps to (x, z, -y), so the shaft axis lands at (x=-1.47, y=14.95).
# TIP_UP=True  -> rpy (+pi/2,0,0): mesh (x,y,z) -> world (x,-z,y), tip points up
# TIP_UP=False -> rpy (-pi/2,0,0): mesh (x,y,z) -> world (x, z,-y), tip points down
TIP_UP = True
PIP_SHAFT_MESH = (-1.47, 14.95)   # shaft axis in mesh (x, z)
PIP_TIP_MESH_Y = 164.1            # +Y end (narrow)
PIP_BUTT_MESH_Y = -217.9          # -Y end
LOW_END_BELOW_SHELF = -60.0       # whichever end is down ends 60 mm under the shelf

if TIP_UP:
    PIP_RPY = (math.pi / 2, 0.0, 0.0)
    PIP_AXIS_AFTER = (PIP_SHAFT_MESH[0], -PIP_SHAFT_MESH[1])
    PIP_Z = LOW_END_BELOW_SHELF - PIP_BUTT_MESH_Y
else:
    PIP_RPY = (-math.pi / 2, 0.0, 0.0)
    PIP_AXIS_AFTER = (PIP_SHAFT_MESH[0], PIP_SHAFT_MESH[1])
    PIP_Z = LOW_END_BELOW_SHELF + PIP_TIP_MESH_Y


def link(name, mesh, xyz, rpy, colour=None):
    o = '<origin xyz="%.5f %.5f %.5f" rpy="%.5f %.5f %.5f"/>' % (xyz + rpy)
    mat = ""
    if colour:
        mat = ('    <material name="%s_mat"><color rgba="%s"/></material>\n' % (name, colour))
    return f"""
  <link name="{name}">
    <visual>
      <geometry><mesh filename="package://pdms_gripper_description/meshes/{mesh}"/></geometry>
      {o}
{mat}    </visual>
    <collision>
      <geometry><mesh filename="package://pdms_gripper_description/meshes/{mesh}"/></geometry>
      {o}
    </collision>
  </link>
"""


def joint(name, parent, child, xyz, rpy, jtype="fixed"):
    return f"""
  <joint name="{name}" type="{jtype}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>
  </joint>
"""


STATION_REPORT = []


def main():
    urdf = open(SRC).read()

    pip_z = PIP_Z   # world z of the pipette mesh origin, above the socket
    parts = []
    parts.append('\n  <link name="world"/>\n')
    parts.append(joint("world_to_base", "world", "base", (0, 0, 0), (0, 0, 0)))
    parts.append(link("frame", "frame_2020.dae", (0, 0, 0), (0, 0, 0), "0.35 0.38 0.40 1"))
    parts.append(joint("world_to_frame", "world", "frame", FRAME_XYZ, (0, 0, FRAME_YAW)))
    # --- stations -----------------------------------------------------------
    # Place each holder by its BORE target (not its flange), so the bore lands
    # exactly on the arc regardless of how much the holder is yawed.
    reach = BORE_IN_BASE[1]
    xs = [(i - (N_STATIONS - 1) / 2.0) * STATION_PITCH for i in range(N_STATIONS)]
    for i, xb in enumerate(xs):
        bore_w = (xb, reach, BORE_IN_BASE[2])
        # clip opening must point from the bore towards the robot base
        d = (-bore_w[0], -bore_w[1])
        dn = math.hypot(*d) or 1.0
        d = (d[0] / dn, d[1] / dn)
        yaw_w = math.atan2(-d[0], d[1])           # local +y -> d
        off = rotz(yaw_w, (0.0, BORE_LOCAL[1] * M, 0.0))
        org_w = tuple(bore_w[k] - off[k] for k in range(3))
        # world -> frame
        rel = tuple(org_w[k] - FRAME_XYZ[k] for k in range(3))
        org_f = rotz(-FRAME_YAW, rel)
        suffix = "" if N_STATIONS == 1 else "_%d" % i
        hname = "pipette_holder" + suffix
        sname = "pipette_socket" + suffix
        parts.append(link(hname, HOLDER_MESH, (0, 0, 0), (0, 0, 0), "0.85 0.45 0.10 1"))
        parts.append(joint("frame_to_" + hname, "frame", hname, org_f,
                           (0, 0, yaw_w - FRAME_YAW)))
        parts.append('\n  <link name="%s"/>\n' % sname)
        parts.append(joint(hname + "_to_" + sname, hname, sname,
                           (BORE_LOCAL[0] * M, BORE_LOCAL[1] * M, 0.0), (0, 0, 0)))
        STATION_REPORT.append((i, bore_w, math.degrees(yaw_w - math.pi / 2)))

    # The pipette is published by tools/pipette_attach.py as a TF frame plus an
    # RViz Marker, not as a URDF link -- a URDF joint's parent is fixed at parse
    # time, so a link could never move from the holder to the gripper.
    # Pass --static-pipette to bake it into the holder instead (no grasping).
    if STATIC_PIPETTE:
        xyz = (-PIP_AXIS_AFTER[0] * M, -PIP_AXIS_AFTER[1] * M, pip_z * M)
        parts.append(link("pipette", "assembled_pipette.dae", xyz, PIP_RPY, "0.15 0.65 0.35 1"))
        parts.append(joint("socket_to_pipette", "pipette_socket", "pipette",
                           (0, 0, 0), (0, 0, 0)))

    urdf = urdf.replace("</robot>", "".join(parts) + "\n</robot>")
    # MoveIt's collision loader (geometric_shapes) ignores the COLLADA
    # <unit meter="0.001"> tag that RViz honours, so collision meshes load
    # 1000x oversized. Make the scale explicit on every collision mesh.
    def scale_collisions(x):
        out, pos = [], 0
        for m in re.finditer(r"<collision>.*?</collision>", x, re.S):
            blk = m.group(0)
            if "scale=" not in blk:
                blk = re.sub(r'(<mesh filename="[^"]+")', r'\1 scale="0.001 0.001 0.001"', blk)
            out.append(x[pos:m.start()]); out.append(blk); pos = m.end()
        out.append(x[pos:])
        return "".join(out)

    urdf = scale_collisions(urdf)

    open(OUT, "w").write(urdf)

    # Single source of truth for the pipette pose. pipette_attach.py reads this
    # so the Marker can never drift out of sync with the URDF again.
    xyz = (-PIP_AXIS_AFTER[0] * M, -PIP_AXIS_AFTER[1] * M, pip_z * M)
    with open(POSE_OUT, "w") as fh:
        fh.write("# generated by build_workcell.py - do not edit\n")
        fh.write("held_xyz: [%.5f, %.5f, %.5f]\n" % xyz)
        fh.write("held_rpy: [%.5f, %.5f, %.5f]\n" % PIP_RPY)
        fh.write("tip_up: %s\n" % ("true" if TIP_UP else "false"))
    print("wrote", POSE_OUT)
    print("  held_xyz", tuple(round(v, 5) for v in xyz), " held_rpy",
          tuple(round(v, 5) for v in PIP_RPY))
    print("wrote", OUT)
    for i, b, fan in STATION_REPORT:
        print("  station %d: bore world (%.3f, %.3f, %.3f)  clip fan %+.1f deg"
              % (i, b[0], b[1], b[2], fan))
    print("  frame origin in base frame :", tuple(round(v, 5) for v in FRAME_XYZ))
    print("  bore in base frame         :", BORE_IN_BASE)
    print("  pipette origin z (mm)      :", round(pip_z, 2))


if __name__ == "__main__":
    main()
