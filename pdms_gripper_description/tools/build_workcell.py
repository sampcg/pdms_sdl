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
OUT = os.path.join(PKG, "urdf", "mycobot_320_pi_2022_workcell.urdf")

STATIC_PIPETTE = "--static-pipette" in sys.argv

M = 0.001  # mesh units are mm

# --- derived geometry (mm) -------------------------------------------------
BAR_FACE_X = 5.87        # +X face of the lower Y crossbar
BAR_CENTRE_Y = -269.72   # midpoint of that bar along its length
BAR_MID_Z = 250.0        # bar mid-height
HOLDER_YAW = -math.pi / 2
BORE_LOCAL = (0.0, 35.0)  # bore centre in holder x,y

# bore position in frame coords, after the holder yaw
BORE_IN_FRAME = (BAR_FACE_X + BORE_LOCAL[1], BAR_CENTRE_Y + BORE_LOCAL[0], BAR_MID_Z)

# where we want the bore to land relative to the robot base (metres)
BORE_IN_BASE = (0.280, 0.0, 0.250)

FRAME_XYZ = tuple(BORE_IN_BASE[i] - BORE_IN_FRAME[i] * M for i in range(3))

# --- pipette ---------------------------------------------------------------
# shaft axis lies along mesh +Y at (x=-1.47, z=14.95); tip is the +Y end.
# rpy=(-pi/2,0,0) sends mesh +Y -> world -Z (tip down); then mesh (x,y,z)
# maps to (x, z, -y), so the shaft axis lands at (x=-1.47, y=14.95).
PIP_RPY = (-math.pi / 2, 0.0, 0.0)
PIP_AXIS_AFTER = (-1.47, 14.95)
PIP_TIP_MESH_Y = 164.1
TIP_BELOW_SHELF = -60.0        # tip ends 60 mm below the holder shelf


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


def main():
    urdf = open(SRC).read()

    pip_z = TIP_BELOW_SHELF + PIP_TIP_MESH_Y   # world z of pipette origin
    parts = []
    parts.append('\n  <link name="world"/>\n')
    parts.append(joint("world_to_base", "world", "base", (0, 0, 0), (0, 0, 0)))
    parts.append(link("frame", "frame_2020.dae", (0, 0, 0), (0, 0, 0), "0.35 0.38 0.40 1"))
    parts.append(joint("world_to_frame", "world", "frame", FRAME_XYZ, (0, 0, 0)))
    parts.append(link("pipette_holder", "pipette_holder_2020.dae", (0, 0, 0), (0, 0, 0),
                      "0.85 0.45 0.10 1"))
    parts.append(joint("frame_to_holder", "frame", "pipette_holder",
                       (BAR_FACE_X * M, BAR_CENTRE_Y * M, BAR_MID_Z * M),
                       (0, 0, HOLDER_YAW)))
    # a frame at the bore centre, so the pipette node has something to hang off
    parts.append('\n  <link name="pipette_socket"/>\n')
    parts.append(joint("holder_to_socket", "pipette_holder", "pipette_socket",
                       (BORE_LOCAL[0] * M, BORE_LOCAL[1] * M, 0.0), (0, 0, 0)))

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
    open(OUT, "w").write(urdf)
    print("wrote", OUT)
    print("  frame origin in base frame :", tuple(round(v, 5) for v in FRAME_XYZ))
    print("  bore in base frame         :", BORE_IN_BASE)
    print("  pipette origin z (mm)      :", round(pip_z, 2))


if __name__ == "__main__":
    main()
