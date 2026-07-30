#!/usr/bin/env python3
"""Pipette as a detachable object.

The pipette is deliberately NOT a URDF link: a URDF joint's parent is fixed at
parse time, so a link can never change parent at runtime. Instead this node
publishes the pipette as a TF frame plus an RViz mesh Marker, and re-parents
that frame between the holder and the gripper on command.

    HELD   pipette_socket -> pipette      (sitting in the holder bore)
    GRASP  gripper_base   -> pipette      (carried by the forks)

Toggle it:

    ros2 topic pub --once /pipette/attach std_msgs/Bool "{data: true}"    # grasp
    ros2 topic pub --once /pipette/attach std_msgs/Bool "{data: false}"   # release

Both offsets are ROS parameters, so you can tune the grasp live:

    ros2 param set /pipette_attach grasp_xyz "[0.0, 0.065, -0.015]"
    ros2 param set /pipette_attach grasp_rpy "[-1.5708, 0.0, 0.0]"
"""
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

MESH = "package://pdms_gripper_description/meshes/assembled_pipette.dae"
MESH_DIR = "package://pdms_gripper_description/meshes/"


def _load_objects():
    """Read config/objects*.yaml if present: one entry per movable object.

    Without it the node behaves exactly as before - a single pipette in a single
    socket - so the plain workcell is unaffected.
    """
    d = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
    for name in ("objects_rail.yaml",):
        p = os.path.join(d, name)
        if not os.path.exists(p):
            continue
        objs, cur = [], None
        for line in open(p):
            line = line.split("#")[0].rstrip()
            if not line.strip():
                continue
            if line.strip().startswith("- name:"):
                cur = {"name": line.split(":", 1)[1].strip()}
                objs.append(cur)
            elif cur is not None and ":" in line:
                k, v = line.strip().split(":", 1)
                v = v.strip()
                if v.startswith("["):
                    cur[k] = [float(x) for x in v.strip("[]").split(",")]
                else:
                    cur[k] = v
        if objs:
            return objs
    return None

# Pose of the pipette mesh origin in its parent frame. These are DEFAULTS only
# -- the real values come from config/pipette_pose.yaml, which build_workcell.py
# generates alongside the URDF so the Marker and the URDF cannot disagree.
HELD_XYZ = [0.00147, 0.01495, 0.1579]
HELD_RPY = [math.pi / 2, 0.0, 0.0]


def _load_pose():
    """Read the generated pose file; fall back to the defaults above."""
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", "pipette_pose.yaml")
    xyz, rpy = list(HELD_XYZ), list(HELD_RPY)
    try:
        for line in open(path):
            line = line.split("#")[0].strip()
            if line.startswith("held_xyz:"):
                xyz = [float(v) for v in line.split("[")[1].split("]")[0].split(",")]
            elif line.startswith("held_rpy:"):
                rpy = [float(v) for v in line.split("[")[1].split("]")[0].split(",")]
    except (IOError, IndexError, ValueError):
        pass
    return xyz, rpy

# In the gripper: between the fork clamping faces. Derived from the stock
# blade clamping-face centroids in gripper_base coords (+-64 mm in x, y~65 mm,
# z~-15 mm), so the midpoint is roughly (0, 0.065, -0.015).
GRASP_XYZ = [0.00147, 0.065, -0.015]
GRASP_RPY = [math.pi / 2, 0.0, 0.0]


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class PipetteAttach(Node):
    def __init__(self):
        super().__init__("pipette_attach")
        self.declare_parameter("held_frame", "pipette_socket")
        self.declare_parameter("grasp_frame", "gripper_base")
        held_xyz, held_rpy = _load_pose()
        self.declare_parameter("held_xyz", held_xyz)
        self.declare_parameter("held_rpy", held_rpy)
        self.declare_parameter("grasp_xyz", GRASP_XYZ)
        self.declare_parameter("grasp_rpy", GRASP_RPY)
        self.declare_parameter("max_grasp_distance", 0.15)

        objs = _load_objects()
        if objs:
            self.objs = objs
        else:
            hx, hr = _load_pose()
            self.objs = [{"name": "pipette", "socket": "pipette_socket",
                          "mesh": "assembled_pipette.dae", "xyz": hx, "rpy": hr}]
        self.n = len(self.objs)
        self.held = [None] * self.n
        self.grasped = -1
        self.attached = False
        # Captured at the instant of the grasp: the pipette's pose relative to
        # gripper_base right then. Using this instead of a hardcoded offset
        # means the pipette is picked up exactly where it stood - no jump, and
        # no guessing where the fork clamping midpoint is.
        self.grasp_tf = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/pipette/marker", 1)
        self.create_subscription(Bool, "/pipette/attach", self.on_attach, 1)
        self.create_subscription(Int32, "/pipette/grasp", self.on_grasp, 1)
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("%d movable object(s): %s"
                               % (self.n, ", ".join(o["name"] for o in self.objs)))
        self.get_logger().info("publish /pipette/grasp (Int32 index, -1 = release)")

    def on_attach(self, msg):
        """Back-compatible single-object toggle: acts on object 0."""
        self.on_grasp(Int32(data=(0 if msg.data else -1)))

    def on_grasp(self, msg):
        i = int(msg.data)
        if i == self.grasped:
            return
        if i >= self.n:
            self.get_logger().warn("no object %d (have %d)" % (i, self.n))
            return
        if i >= 0:
            self.held[i] = self.capture_grasp(self.objs[i]["name"])
        self.grasped = i
        self.attached = i >= 0
        self.get_logger().info(
            "%s" % ("GRASPED %s (#%d)" % (self.objs[i]["name"], i) if i >= 0 else "RELEASED"))

    def capture_grasp(self, child):
        """Pose of `child` in `gripper_base` at this moment, via TF."""
        import rclpy.time
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter("grasp_frame").value, child,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn("grasp capture failed (%s); using default offset" % e)
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        d = math.sqrt(tr.x ** 2 + tr.y ** 2 + tr.z ** 2)
        # Sanity check: the capture is only meaningful if the gripper has
        # actually reached the pipette. If /pipette/attach fires early (e.g. the
        # timeline was reordered or `approach` was lengthened), we would freeze a
        # far-away offset and the pipette would fly along beside the gripper.
        limit = float(self.get_parameter("max_grasp_distance").value)
        if d > limit:
            self.get_logger().error(
                "grasp captured %.0f mm from gripper_base (limit %.0f mm) - the "
                "gripper is probably not at the pipette yet; check when "
                "/pipette/attach fires in the timeline" % (d * 1000, limit * 1000))
        else:
            self.get_logger().info("  captured grasp offset xyz=(%.4f, %.4f, %.4f)  |d|=%.0f mm"
                                   % (tr.x, tr.y, tr.z, d * 1000))
        return ([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])

    def _p(self, n):
        return list(self.get_parameter(n).value)

    def tick(self):
        gframe = self.get_parameter("grasp_frame").value
        for i, o in enumerate(self.objs):
            child = o["name"]
            if i == self.grasped and self.held[i] is not None:
                xyz, quat = self.held[i]
                self.send(gframe, xyz, quat, child)
            else:
                self.send(o["socket"], o["xyz"],
                          quat_from_rpy(*[float(v) for v in o["rpy"]]), child)
            self.publish_marker(child, i, o["mesh"])

    def send(self, parent, xyz, quat, child="pipette"):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x, t.transform.rotation.y = float(quat[0]), float(quat[1])
        t.transform.rotation.z, t.transform.rotation.w = float(quat[2]), float(quat[3])
        self.br.sendTransform(t)

    def publish_marker(self, child="pipette", idx=0, mesh="assembled_pipette.dae"):
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = child
        m.header.stamp = now
        m.ns = "pipette"
        m.id = idx
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = MESH_DIR + mesh
        m.mesh_use_embedded_materials = False
        m.scale.x = m.scale.y = m.scale.z = 1.0
        cols = [(0.15, 0.65, 0.35), (0.10, 0.55, 0.70), (0.80, 0.35, 0.35)]
        m.color.r, m.color.g, m.color.b = cols[idx % len(cols)]
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        self.marker_pub.publish(m)


def main():
    rclpy.init()
    n = PipetteAttach()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
