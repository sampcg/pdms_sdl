#!/usr/bin/env python3
"""Pick-and-place through MoveIt, so every motion is collision-checked.

The difference from pdms_gripper_description/tools/workcell_sequence.py is what
happens between waypoints. That node interpolates a spline and publishes joint
states directly - fast, smooth, and completely unaware that the frame, the rail
and the holders exist. This node sends each waypoint to MoveGroup, so OMPL plans
a path that is checked against the SRDF collision matrix: the arm against
itself, and the arm against the workcell.

IK still comes from the same Chain class, because it lets us pin the approach
azimuth head-on; MoveIt then plans between the resulting joint configurations.

Needs move_group running:

    ros2 launch pdms_320_moveit pick_place.launch.py

Parameters:
    station      0 = left syringe, 1 = right syringe, 2 = mixer   (default 0)
    loop         repeat forever                                    (default True)
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)
from std_msgs.msg import Int32

ARM = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
       "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

# Not 0.0 / the joint limit: OMPL cannot sample a goal region that sits exactly
# on a bound, so leave a little room at each end.
GRIP_OPEN = -0.05
GRIP_TRANSIT = -0.65
GRIP_CLOSED = -0.78

# station -> (bore x, bore y, grasp height). The mixer is gripped lower because
# its holder occupies the band around its centre of mass.
STATIONS = {
    0: (-0.115, -0.410, 0.260),
    1: (0.115, -0.410, 0.260),
    2: (0.000, -0.410, 0.217),
}


def find_chain():
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("pdms_gripper_description")
    sys.path.insert(0, os.path.join(share, "tools"))
    import workcell_sequence as W
    return W, os.path.join(share, "urdf", "mycobot_320_pi_2022_workcell_rail.urdf")


class PickPlace(Node):
    def __init__(self):
        super().__init__("moveit_pick_place")
        self.declare_parameter("station", 0)
        self.declare_parameter("loop", True)

        W, urdf = find_chain()
        self.chain = W.Chain(urdf)

        self.ac = ActionClient(self, MoveGroup, "/move_action")
        self.grasp_pub = self.create_publisher(Int32, "/pipette/grasp", 1)
        self.get_logger().info("waiting for move_group...")
        if not self.ac.wait_for_server(timeout_sec=90.0):
            raise RuntimeError("move_group never appeared - is the launch up?")
        self.station = int(self.get_parameter("station").value)
        self.wp = self.solve()

    # ------------------------------------------------------------------ IK ---
    @staticmethod
    def R_azimuth(phi):
        c, s = math.cos(phi), math.sin(phi)
        return np.array([[-s, c, 0.0], [c, s, 0.0], [0.0, 0.0, -1.0]])

    def solve_pose(self, p, q0):
        """Prefer a head-on approach; widen only if nothing there is reachable."""
        phi0 = math.atan2(p[1], p[0])
        seeds = [q0, np.array([0.0, -0.6, 1.2, -0.6, 0.0, 0.0])]
        offs = [0.0] + [math.radians(d) for d in
                        (5, -5, 10, -10, 15, -15, 20, -20, 25, -25)]
        fallback = (1e9, q0)
        for off in offs:
            R = self.R_azimuth(phi0 + off)
            for s in seeds:
                q, e = self.chain.ik(p, R, s)
                if e < 3e-3:
                    return q.copy(), e, math.degrees(off)
                if e < fallback[0]:
                    fallback = (e, q.copy())
        return fallback[1], fallback[0], None

    def solve(self):
        bx, by, gh = STATIONS[self.station]
        grasp = np.array([bx, by, gh])
        ap = np.array([bx, by, 0.0])
        ap = ap / (np.linalg.norm(ap) or 1.0)
        targets = [
            ("pregrasp", grasp - ap * 0.085),
            ("grasp", grasp),
            ("extract", grasp - ap * 0.090),      # horizontal pull, towards the robot
            ("via", np.array([0.190, -0.190, 0.330])),
        ]
        out = {}
        q = np.array([0.0, -0.6, 1.2, -0.6, 0.0, 0.0])
        self.get_logger().info("station %d, bore (%.3f, %.3f), grasp z %.3f"
                               % (self.station, bx, by, gh))
        for name, p in targets:
            q, e, off = self.solve_pose(p, q)
            out[name] = q.copy()
            self.get_logger().info(
                "  %-9s %s  residual %.1f mm  azimuth %s"
                % (name, np.round(p, 3).tolist(), e * 1000,
                   ("%+.0f deg" % off) if off is not None else "off-axis"))
        return out

    # -------------------------------------------------------------- MoveIt ---
    def move(self, joints, group, label):
        req = MotionPlanRequest()
        req.group_name = group
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 10
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4
        c = Constraints()
        for k, v in joints.items():
            c.joint_constraints.append(JointConstraint(
                joint_name=k, position=float(v),
                tolerance_above=0.02, tolerance_below=0.02, weight=1.0))
        req.goal_constraints = [c]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)

        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("%s: goal rejected" % label)
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=180.0)
        res = rf.result()
        code = res.result.error_code.val if res else -99
        ok = code == 1
        # -12 / -31 are the collision and IK failures worth naming explicitly
        why = {1: "ok", -1: "FAILED", -12: "PLAN IN COLLISION",
               -31: "NO IK SOLUTION", -99: "TIMEOUT"}.get(code, "code=%d" % code)
        (self.get_logger().info if ok else self.get_logger().error)(
            "%-14s %s" % (label, why))
        return ok

    def grasp(self, idx):
        self.grasp_pub.publish(Int32(data=int(idx)))
        for _ in range(6):
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_once(self):
        arm = lambda k: dict(zip(ARM, self.wp[k]))
        steps = [
            ("open gripper", lambda: self.move({"gripper_controller": GRIP_TRANSIT}, "gripper", "open gripper")),
            ("go to station", lambda: self.move(arm("pregrasp"), "arm_group", "go to station")),
            ("approach", lambda: self.move(arm("grasp"), "arm_group", "approach")),
            ("close gripper", lambda: self.move({"gripper_controller": GRIP_CLOSED}, "gripper", "close gripper")),
            ("pick up", lambda: (self.grasp(self.station),
                                 self.move(arm("extract"), "arm_group", "pick up"))[1]),
            ("carry", lambda: self.move(arm("via"), "arm_group", "carry")),
            ("bring back", lambda: self.move(arm("extract"), "arm_group", "bring back")),
            ("insert", lambda: self.move(arm("grasp"), "arm_group", "insert")),
            ("release", lambda: (self.move({"gripper_controller": GRIP_TRANSIT}, "gripper", "release"),
                                 self.grasp(-1))[0]),
            ("retreat", lambda: self.move(arm("pregrasp"), "arm_group", "retreat")),
        ]
        for _, fn in steps:
            if not fn():
                self.get_logger().error("aborted - see the failure above")
                return False
        self.get_logger().info("cycle complete")
        return True


def main():
    rclpy.init()
    n = PickPlace()
    try:
        while rclpy.ok():
            n.run_once()
            if not n.get_parameter("loop").value:
                break
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
