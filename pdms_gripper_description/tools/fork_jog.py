#!/usr/bin/env python3
"""Live jog panel for the fork finger origins.

Sliders drive the <origin xyz rpy> of gripper_left1 / gripper_right1, rebuild the
URDF in memory, and push it to the running robot_state_publisher, so RViz updates
immediately. No relaunch, no rebuild.

Usage (with the RViz launch already running in another terminal):

    source /home/samuel/colcon_ws/install/setup.bash
    ros2 run pdms_gripper_description fork_jog.py   # or: python3 <share>/tools/fork_jog.py

Optional:  --model /path/to/some.urdf     (defaults to the forkfit URDF)

"Copy values" puts the current origins on the clipboard and also writes them
to  ~/fork_origin.txt
"""
import argparse
import os
import re
import subprocess
import sys
import tkinter as tk
from tkinter import ttk

PKG = "pdms_gripper_description"
SHARE_FALLBACK = "/home/samuel/colcon_ws/install/pdms_gripper_description/share/pdms_gripper_description"
OUT = os.path.join(os.path.expanduser("~"), "fork_origin.txt")

# starting point = the solved forkfit pose
START = {
    "left":  dict(x=-1.28, y=-7.99, z=-0.32, roll=180.0, pitch=0.0, yaw=90.0),
    "right": dict(x=0.00,  y=0.00,  z=0.10,  roll=180.0, pitch=0.0, yaw=90.0),
}
LIMITS = dict(x=(-120, 120), y=(-120, 120), z=(-120, 120),
              roll=(-180, 180), pitch=(-180, 180), yaw=(-180, 180))


def share_dir():
    try:
        p = subprocess.run(["ros2", "pkg", "prefix", "--share", PKG],
                           capture_output=True, text=True, timeout=20)
        if p.returncode == 0 and p.stdout.strip():
            return p.stdout.strip()
    except Exception:
        pass
    return SHARE_FALLBACK


class Jog:
    def __init__(self, model):
        self.model = model
        with open(model) as f:
            self.base = f.read()
        self.vals = {s: dict(START[s]) for s in ("left", "right")}
        self.linked = True
        self.node = None
        self._init_ros()
        self._build_gui()

    # ---------------------------------------------------------------- ROS ---
    def _init_ros(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            rclpy.init(args=None)
            self.rclpy = rclpy
            self.node = Node("fork_jog")
            self.cli = self.node.create_client(SetParameters,
                                               "/robot_state_publisher/set_parameters")
            self.SetParameters = SetParameters
            self.Parameter = Parameter
            self.ParameterValue = ParameterValue
            self.ParameterType = ParameterType
            self.ok = self.cli.wait_for_service(timeout_sec=5.0)
            if not self.ok:
                print("!! /robot_state_publisher not reachable - is the launch running?")
        except Exception as e:                                  # rclpy missing etc.
            print("!! ROS init failed (%s); running in preview-only mode" % e)
            self.ok = False

    def push(self, urdf):
        if not self.ok:
            return
        req = self.SetParameters.Request()
        pv = self.ParameterValue()
        pv.type = self.ParameterType.PARAMETER_STRING
        pv.string_value = urdf
        p = self.Parameter()
        p.name = "robot_description"
        p.value = pv
        req.parameters = [p]
        fut = self.cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)

    # --------------------------------------------------------------- URDF ---
    def urdf(self):
        out = self.base
        for side in ("left", "right"):
            v = self.vals[side]
            xyz = "%.5f %.5f %.5f" % (v["x"] / 1000.0, v["y"] / 1000.0, v["z"] / 1000.0)
            import math
            rpy = "%.5f %.5f %.5f" % (math.radians(v["roll"]), math.radians(v["pitch"]),
                                      math.radians(v["yaw"]))
            m = re.search(r'<link name="gripper_%s1">.*?</link>' % side, out, re.S)
            if not m:
                continue
            blk = re.sub(r'<origin xyz\s*=\s*"[^"]*"\s*rpy\s*=\s*"[^"]*"\s*/>',
                         '<origin xyz = "%s" rpy = "%s"/>' % (xyz, rpy), m.group(0))
            out = out[:m.start()] + blk + out[m.end():]
        return out

    def report(self):
        lines = ["fork origins", "model: " + self.model]
        import math
        for side in ("left", "right"):
            v = self.vals[side]
            lines.append("  gripper_%s1:" % side)
            lines.append("    xyz_mm  = %8.2f %8.2f %8.2f" % (v["x"], v["y"], v["z"]))
            lines.append("    rpy_deg = %8.2f %8.2f %8.2f" % (v["roll"], v["pitch"], v["yaw"]))
            lines.append('    <origin xyz = "%.5f %.5f %.5f" rpy = "%.5f %.5f %.5f"/>' % (
                v["x"] / 1000, v["y"] / 1000, v["z"] / 1000,
                math.radians(v["roll"]), math.radians(v["pitch"]), math.radians(v["yaw"])))
        return "\n".join(lines)

    # ---------------------------------------------------------------- GUI ---
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("fork jog  -  gripper_left1 / gripper_right1")
        self.scales = {}
        self.labels = {}

        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")
        self.link_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="mirror right = left (x negated)",
                        variable=self.link_var).pack(side="left")
        ttk.Button(top, text="Reset", command=self.reset).pack(side="right")
        ttk.Button(top, text="Copy values", command=self.copy).pack(side="right", padx=6)

        for side in ("left", "right"):
            fr = ttk.LabelFrame(self.root, text="gripper_%s1" % side, padding=6)
            fr.pack(fill="x", padx=6, pady=4)
            for k in ("x", "y", "z", "roll", "pitch", "yaw"):
                row = ttk.Frame(fr)
                row.pack(fill="x")
                unit = "mm" if k in "xyz" else "deg"
                ttk.Label(row, text="%-6s" % k, width=7).pack(side="left")
                lo, hi = LIMITS[k]
                var = tk.DoubleVar(value=self.vals[side][k])
                sc = ttk.Scale(row, from_=lo, to=hi, variable=var, orient="horizontal",
                               command=lambda _v, s=side, kk=k: self.on_change(s, kk))
                sc.pack(side="left", fill="x", expand=True, padx=4)
                lab = ttk.Label(row, text="%8.2f %s" % (self.vals[side][k], unit), width=12)
                lab.pack(side="left")
                self.scales[(side, k)] = var
                self.labels[(side, k)] = (lab, unit)

        self.status = ttk.Label(self.root, text="ready", padding=6)
        self.status.pack(fill="x")
        self.out = tk.Text(self.root, height=9, width=74)
        self.out.pack(fill="both", padx=6, pady=4)
        self.push_now()

    def on_change(self, side, k):
        self.vals[side][k] = float(self.scales[(side, k)].get())
        if self.link_var.get() and side == "left":
            other = -self.vals["left"][k] if k in ("x", "yaw") else self.vals["left"][k]
            self.vals["right"][k] = other
            self.scales[("right", k)].set(other)
            self._label("right", k)
        self._label(side, k)
        self.push_now()

    def _label(self, side, k):
        lab, unit = self.labels[(side, k)]
        lab.config(text="%8.2f %s" % (self.vals[side][k], unit))

    def reset(self):
        for side in ("left", "right"):
            for k, v in START[side].items():
                self.vals[side][k] = v
                self.scales[(side, k)].set(v)
                self._label(side, k)
        self.push_now()

    def push_now(self):
        try:
            self.push(self.urdf())
            self.status.config(text="pushed to robot_state_publisher" if self.ok
                               else "preview only (no ROS connection)")
        except Exception as e:
            self.status.config(text="push failed: %s" % e)
        self.out.delete("1.0", "end")
        self.out.insert("1.0", self.report())

    def copy(self):
        txt = self.report()
        with open(OUT, "w") as f:
            f.write(txt + "\n")
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(txt)
        except Exception:
            pass
        print("\n" + txt + "\n-> " + OUT)
        self.status.config(text="copied to clipboard and written to " + OUT)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None)
    a = ap.parse_args()
    model = a.model or os.path.join(
        share_dir(), "urdf/mycobot_320_pi_2022_fork_gripper.urdf")
    if not os.path.exists(model):
        sys.exit("model not found: " + model)
    print("jogging:", model)
    Jog(model).run()
