#!/usr/bin/env python3
"""Timed pick-and-place: go to the syringe, pick it up, carry it, put it down.

This node OWNS /joint_states, so run it *instead of* joint_state_publisher_gui
(the launch file does that for you with sequence:=true). Two publishers on
/joint_states will fight and nothing will move.

It solves IK itself (damped least squares over the six arm joints, using FK
parsed straight out of the URDF), interpolates between the resulting joint
waypoints with a smoothstep ease, and fires /pipette/attach at the exact
moment the forks close.

    ros2 launch pdms_gripper_description workcell.launch.py sequence:=true

Parameters:
    loop          (bool)  repeat forever                      default True
    speed         (float) time multiplier, >1 = slower        default 1.0
    grasp_height  (float) where on the barrel to grip, metres default 0.45
    place_xyz     (float[3]) where to put it down             default below
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

ARM_JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
              "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
TOOL_LINK = "gripper_base"
# Grasp point in gripper_base coords: midpoint between the FORK clamping faces,
# measured from the fork meshes at the closed gripper value:
#   left  face centroid  x=-22.1  y=104.8  z=-15.3   (946 mm2)
#   right face centroid  x=+21.5  y=104.1  z=-14.8   (1072 mm2)
# The old value (y=0.065) came from the STOCK blades, before the forks were
# fitted. It was 39 mm too shallow, so IK drove the gripper past the pipette and
# held the barrel in the throat instead of between the fork faces.
TOOL_OFFSET = np.array([0.0, 0.1044, -0.015])

# Fork gap vs gripper_controller, measured from the meshes at the grasp pose:
#   -0.05  gap 109 mm   outer envelope 199 mm
#   -0.65  gap  50 mm   outer envelope 140 mm   <- clears a 35 mm barrel
#   -0.78  gap  35 mm   outer envelope 127 mm   <- first contact on the barrel
# -0.55 (the old "closed") leaves a 61 mm gap and never touches the barrel.
GRIPPER_OPEN = -0.05        # wide; only needed at home
GRIPPER_TRANSIT = -0.65     # narrow but still clears the barrel
GRIPPER_CLOSED = -0.78      # actually grips a 35 mm barrel

# --- the timeline. (label, duration_s, waypoint_key, gripper, attached) ------
TIMELINE = [
    ("start",           1.0, "pregrasp",  GRIPPER_TRANSIT, False),
    ("approach",        1.5, "grasp",     GRIPPER_TRANSIT, False),
    ("close gripper",   1.0, "grasp",     GRIPPER_CLOSED,  False),
    ("pick up",         1.5, "lift",      GRIPPER_CLOSED,  True),
    ("carry away",      2.0, "via",       GRIPPER_CLOSED,  True),
    ("bring back",      2.0, "lift",      GRIPPER_CLOSED,  True),
    ("insert",          1.5, "grasp",     GRIPPER_CLOSED,  True),
    ("release",         1.0, "grasp",     GRIPPER_TRANSIT, False),
    ("retreat",         1.5, "pregrasp",  GRIPPER_TRANSIT, False),
    ("home",            2.0, "home",      GRIPPER_OPEN,    False),
    ("return",          1.5, "pregrasp",  GRIPPER_TRANSIT, False),
]


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def axis_rot(axis, a):
    ax = axis / (np.linalg.norm(axis) or 1.0)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


class Chain:
    """Serial-chain FK/IK read straight from the URDF."""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        order = []
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0").split())]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0").split())]) if o is not None else np.zeros(3)
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([0, 0, 1.0])
            self.joints[j.get("name")] = dict(
                xyz=xyz, rpy=rpy, axis=axis, type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"))
            order.append(j.get("name"))
        # fixed hop from the last arm joint's child to the tool link
        self.tail = [n for n, d in self.joints.items()
                     if d["type"] == "fixed" and d["child"] == TOOL_LINK]

    def fk(self, q):
        T = np.eye(4)
        for name, a in zip(ARM_JOINTS, q):
            d = self.joints[name]
            L = np.eye(4)
            L[:3, :3] = rpy_mat(*d["rpy"])
            L[:3, 3] = d["xyz"]
            R = np.eye(4)
            R[:3, :3] = axis_rot(d["axis"], a)
            T = T @ L @ R
        for name in self.tail:
            d = self.joints[name]
            L = np.eye(4)
            L[:3, :3] = rpy_mat(*d["rpy"])
            L[:3, 3] = d["xyz"]
            T = T @ L
        return T

    def tool(self, q):
        T = self.fk(q)
        return T[:3, 3] + T[:3, :3] @ TOOL_OFFSET, T[:3, :3]

    def ik(self, p_target, R_target, q0, iters=120, lam=0.08, w_rot=0.45):
        q = np.array(q0, float)
        for _ in range(iters):
            p, R = self.tool(q)
            ep = p_target - p
            Re = R_target @ R.T
            ang = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
            if abs(ang) < 1e-9:
                er = np.zeros(3)
            else:
                er = ang / (2 * math.sin(ang)) * np.array(
                    [Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            e = np.concatenate([ep, w_rot * er])
            if np.linalg.norm(ep) < 5e-4 and abs(ang) < 0.02:
                break
            J = np.zeros((6, 6))
            d = 1e-6
            for i in range(6):
                qd = q.copy()
                qd[i] += d
                pd, Rd = self.tool(qd)
                J[:3, i] = (pd - p) / d
                Rr = Rd @ R.T
                a2 = math.acos(max(-1.0, min(1.0, (np.trace(Rr) - 1) / 2)))
                if abs(a2) < 1e-12:
                    v = np.zeros(3)
                else:
                    v = a2 / (2 * math.sin(a2)) * np.array(
                        [Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]])
                J[3:, i] = w_rot * v / d
            JT = J.T
            q = q + JT @ np.linalg.solve(J @ JT + (lam ** 2) * np.eye(6), e)
            q = np.clip(q, -3.0, 3.0)
        p, R = self.tool(q)
        return q, np.linalg.norm(p_target - p)


def smoothstep(t):
    return t * t * (3 - 2 * t)


def catmull_rom(p0, p1, p2, p3, u):
    """C1-continuous spline through p1->p2. Using this instead of a per-segment
    smoothstep is what removes the stop-start: velocity no longer drops to zero
    at every waypoint, it carries through from the previous segment."""
    u2, u3 = u * u, u * u * u
    return 0.5 * ((2 * p1) +
                  (-p0 + p2) * u +
                  (2 * p0 - 5 * p1 + 4 * p2 - p3) * u2 +
                  (-p0 + 3 * p1 - 3 * p2 + p3) * u3)


class Sequence(Node):
    def __init__(self):
        super().__init__("workcell_sequence")
        share = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.declare_parameter("urdf", os.path.join(
            share, "urdf", "mycobot_320_pi_2022_workcell.urdf"))
        self.declare_parameter("loop", True)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("grasp_height", 0.260)   # holder plate top is now 0.225
        self.declare_parameter("place_xyz", [0.190, -0.190, 0.230])
        self.declare_parameter("bore_xy", [0.0, -0.410])
        # how far off head-on the grasp azimuth may stray, degrees
        self.declare_parameter("azimuth_tolerance_deg", 25.0)

        urdf = self.get_parameter("urdf").value
        self.chain = Chain(urdf)

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.attach_pub = self.create_publisher(Bool, "/pipette/attach", 1)

        self.wp = self.solve_waypoints()
        self.total = sum(d for _, d, _, _, _ in TIMELINE)
        self.t0 = self.get_clock().now()
        self.last_attached = None
        self.last_label = None
        self.create_timer(1.0 / 50.0, self.tick)
        self.get_logger().info("sequence ready; %.1f s per cycle" % self.total)

    # ---- pose targets -----------------------------------------------------
    @staticmethod
    def R_azimuth(phi):
        """gripper_base orientation approaching a vertical shaft from azimuth phi.

        +Y of gripper_base (the approach direction) points along
        (cos phi, sin phi, 0); +X (the clamping direction) is horizontal and
        perpendicular; +Z then always falls to -Z world.
        """
        c, s = math.cos(phi), math.sin(phi)
        return np.array([[-s, c, 0.0],
                         [c,  s, 0.0],
                         [0.0, 0.0, -1.0]])

    def preferred_phi(self):
        """Azimuth for a head-on grasp: the gripper approaches along the
        base->bore direction, so the forks close on the pipette front-on rather
        than from an arbitrary side."""
        bore = list(self.get_parameter("bore_xy").value)
        return math.atan2(bore[1], bore[0])

    def solve_pose(self, p, q0):
        """IK, preferring a head-on approach.

        The azimuth about a vertical shaft is geometrically free, but not all
        choices look right - a side or rear approach is valid IK and a bad
        grasp. So search a narrow window around the head-on direction first and
        only widen if nothing there is reachable.
        """
        seeds = [q0, np.array([0.0, -0.6, 1.2, -0.6, 0.0, 0.0])]
        phi0 = self.preferred_phi()
        tol = math.radians(float(self.get_parameter("azimuth_tolerance_deg").value))

        # pass 1: head-on, then progressively off-axis but still frontal
        window = [0.0]
        d = math.radians(5)
        while d <= tol:
            window += [d, -d]
            d += math.radians(5)
        # Take the FIRST offset that is accurate enough, walking outwards from
        # head-on. Taking the lowest-residual one instead would pick essentially
        # at random, since every azimuth here solves to about the same 1 mm.
        ok = 3e-3
        for stage, offsets in enumerate((window,
                                         sorted(np.arange(-math.pi, math.pi, math.radians(15)),
                                                key=abs))):
            fallback = (1e9, None)
            for off in offsets:
                R = self.R_azimuth(phi0 + off)
                for s in seeds:
                    q, err = self.chain.ik(p, R, s)
                    if err < ok:
                        if stage == 1 and abs(off) > tol:
                            self.get_logger().warn(
                                "   head-on not reachable; using %+.0f deg off"
                                % math.degrees(off))
                        return q.copy(), err
                    if err < fallback[0]:
                        fallback = (err, q.copy())
        return fallback[1], fallback[0]

    def solve_waypoints(self):
        bore = self.get_parameter("bore_xy").value
        gh = float(self.get_parameter("grasp_height").value)
        place = list(self.get_parameter("place_xyz").value)

        grasp = np.array([bore[0], bore[1], gh])
        approach = np.array([bore[0], bore[1], 0.0])
        approach = approach / (np.linalg.norm(approach) or 1.0)   # base -> bore
        targets = {
            # derived rest pose: backed off along the approach axis and raised.
            # Placed at the END of the cycle so the arm still starts by moving
            # towards the pipette, never away from it.
            "home":     grasp - approach * 0.20 + np.array([0, 0, 0.08]),
            "pregrasp": grasp - approach * 0.085,
            "grasp":    grasp,
            "lift":     grasp - approach * 0.02 + np.array([0, 0, 0.09]),
            # Where the pipette is carried to before being brought back.
            # It is NOT set down here - the pipette is returned to the holder
            # bore and only released once seated, so the release never leaves
            # it hanging in mid-air.
            "via":      np.array(place) + np.array([0, 0, 0.10]),
        }
        # No "home" waypoint at the start: the cycle begins at pregrasp, so the
        # arm never reverses away from the pipette before approaching it.
        out = {}
        q = np.array([0.0, -0.6, 1.2, -0.6, 0.0, 0.0])   # IK seed only
        for k in ["home", "pregrasp", "grasp", "lift", "via"]:
            q, err = self.solve_pose(targets[k], q)
            out[k] = q.copy()
            off = math.degrees(self._phi_of(q) - self.preferred_phi())
            off = (off + 180) % 360 - 180
            msg = "  %-9s target %s  residual %.1f mm  azimuth %+.0f deg off head-on" % (
                k, np.round(targets[k], 3).tolist(), err * 1000, off)
            if err < 3e-3:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg + "   <-- NOT REACHED")

        # Straight-line Cartesian sub-waypoints where the shape of the path
        # actually matters: sliding into the pipette and lifting it clear.
        # Joint-space interpolation between two poses bows outward; sampling the
        # straight line and solving IK at each sample keeps the tool on it.
        self.sub = {}
        for name, a, b, n in (("approach", "pregrasp", "grasp", 6),
                              ("pick up",  "grasp", "lift", 6),
                              ("insert",   "lift", "grasp", 6)):
            pa, pb = targets[a], targets[b]
            qs, qi = [], out[a].copy()
            for i in range(1, n):
                p = pa + (pb - pa) * (i / float(n))
                qi, e = self.chain.ik(p, self.R_azimuth(self._phi_of(out[a])), qi)
                qs.append(qi.copy())
            self.sub[name] = qs
            self.get_logger().info("  %-9s %d cartesian sub-points" % (name, len(qs)))
        return out

    def _phi_of(self, q):
        """Approach azimuth actually used by a solved configuration."""
        _, R = self.chain.tool(q)
        return math.atan2(R[1, 1], R[0, 1])

    # ---- playback ---------------------------------------------------------
    def segment(self, idx, prev_key, key, u):
        """Position along one timeline segment.

        Segments with Cartesian sub-points (approach, pick up) walk the
        straight-line samples. Everything else uses a Catmull-Rom spline whose
        control points come from the neighbouring waypoints, so velocity is
        continuous across waypoint boundaries instead of dropping to zero.
        """
        lab = TIMELINE[idx][0]
        sub = self.sub.get(lab)
        if sub:
            pts = [self.wp[prev_key]] + sub + [self.wp[key]]
            s = u * (len(pts) - 1)
            i = min(int(s), len(pts) - 2)
            return (1 - (s - i)) * pts[i] + (s - i) * pts[i + 1]

        if prev_key == key:                      # gripper-only step, arm holds
            return self.wp[key]

        keys = [seg[2] for seg in TIMELINE]
        p1, p2 = self.wp[prev_key], self.wp[key]
        p0 = self.wp[keys[idx - 1]] if idx - 1 >= 0 else p1
        p3 = self.wp[keys[idx + 1]] if idx + 1 < len(keys) else p2
        return catmull_rom(p0, p1, p2, p3, u)

    def tick(self):
        speed = max(0.05, float(self.get_parameter("speed").value))
        el = (self.get_clock().now() - self.t0).nanoseconds / 1e9 / speed
        if el >= self.total:
            if not self.get_parameter("loop").value:
                el = self.total - 1e-3
            else:
                self.t0 = self.get_clock().now()
                el = 0.0

        acc = 0.0
        prev_key = TIMELINE[0][2]
        prev_grip = TIMELINE[0][3]
        label, q, grip, attached = None, None, None, False
        for idx, (lab, dur, key, g, att) in enumerate(TIMELINE):
            if el <= acc + dur:
                u = min(1.0, max(0.0, (el - acc) / dur))
                q = self.segment(idx, prev_key, key, u)
                grip = (1 - smoothstep(u)) * prev_grip + smoothstep(u) * g
                label, attached = lab, att
                break
            acc += dur
            prev_key, prev_grip = key, g
        if q is None:
            q, grip, attached, label = self.wp["pregrasp"], GRIPPER_TRANSIT, False, "start"

        if attached != self.last_attached:
            self.attach_pub.publish(Bool(data=bool(attached)))
            self.last_attached = attached
        if label != self.last_label:
            self.get_logger().info("[%5.1fs] %s" % (el, label))
            self.last_label = label

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ARM_JOINTS + ["gripper_controller"]
        m.position = [float(v) for v in q] + [float(grip)]
        self.pub.publish(m)


def main():
    rclpy.init()
    n = Sequence()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
