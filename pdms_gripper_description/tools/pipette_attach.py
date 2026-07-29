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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

MESH = "package://pdms_gripper_description/meshes/assembled_pipette.dae"

# Pose of the pipette mesh origin in its parent frame.
# In the holder: shaft axis lands on the bore centre once mesh +Y is rotated to
# -Z; the tip then ends 60 mm below the shelf.
HELD_XYZ = [0.00147, -0.01495, 0.1041]
HELD_RPY = [-math.pi / 2, 0.0, 0.0]

# In the gripper: between the fork clamping faces. Derived from the stock
# blade clamping-face centroids in gripper_base coords (+-64 mm in x, y~65 mm,
# z~-15 mm), so the midpoint is roughly (0, 0.065, -0.015).
GRASP_XYZ = [0.00147, 0.065, -0.015]
GRASP_RPY = [-math.pi / 2, 0.0, 0.0]


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
        self.declare_parameter("held_xyz", HELD_XYZ)
        self.declare_parameter("held_rpy", HELD_RPY)
        self.declare_parameter("grasp_xyz", GRASP_XYZ)
        self.declare_parameter("grasp_rpy", GRASP_RPY)

        self.attached = False
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/pipette/marker", 1)
        self.create_subscription(Bool, "/pipette/attach", self.on_attach, 1)
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("pipette in holder; publish /pipette/attach to grasp")

    def on_attach(self, msg):
        if msg.data != self.attached:
            self.attached = msg.data
            self.get_logger().info("pipette %s" % ("GRASPED" if msg.data else "RELEASED"))

    def _p(self, n):
        return list(self.get_parameter(n).value)

    def tick(self):
        if self.attached:
            parent = self.get_parameter("grasp_frame").value
            xyz, rpy = self._p("grasp_xyz"), self._p("grasp_rpy")
        else:
            parent = self.get_parameter("held_frame").value
            xyz, rpy = self._p("held_xyz"), self._p("held_rpy")

        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = parent
        t.child_frame_id = "pipette"
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        q = quat_from_rpy(*[float(v) for v in rpy])
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.br.sendTransform(t)

        m = Marker()
        m.header.frame_id = "pipette"
        m.header.stamp = now
        m.ns = "pipette"
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = MESH
        m.mesh_use_embedded_materials = False
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.65, 0.35, 1.0
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
