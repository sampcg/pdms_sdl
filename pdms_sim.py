#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class PDMSWorkflow(Node):
    def __init__(self):
        super().__init__('pdms_workflow')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = ['joint2_to_joint1', 'joint3_to_joint2',
                            'joint4_to_joint3', 'joint5_to_joint4',
                            'joint6_to_joint5', 'joint6output_to_joint6']

    def move_to(self, angles_deg, label="", duration=2.0):
        """Smoothly interpolate to a pose over `duration` seconds."""
        self.get_logger().info(f"→ {label}: {angles_deg}")
        steps = int(duration * 20)  # 20 Hz
        current = getattr(self, 'last_pose', [0.0]*6)
        target = [math.radians(a) for a in angles_deg]
        for i in range(1, steps + 1):
            t = i / steps
            interp = [c + (tgt - c) * t for c, tgt in zip(current, target)]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = interp
            self.pub.publish(msg)
            time.sleep(1.0 / 20)
        self.last_pose = target

    def run_workflow(self):
        # Placeholder poses — degrees per joint [j1, j2, j3, j4, j5, j6]
        # Replace with your real teach-recorded values later.
        poses = {
            "home":            [0,    0,   0,    0,   0,   0],
            "tip_pickup":      [45,  -30, -30,  -30,  0,   0],
            "base_aspirate":   [90,  -45, -20,  -25,  0,   0],
            "mixing_dispense": [0,   -40, -30,  -20,  0,   0],
            "curing_aspirate": [-90, -45, -20,  -25,  0,   0],
            "mixing_dispense2":[0,   -40, -30,  -20,  0,   0],
            "waste_eject":     [135, -20, -30,  -30,  0,   0],
            "home_end":        [0,    0,   0,    0,   0,   0],
        }
        for label, angles in poses.items():
            self.move_to(angles, label, duration=2.5)
            time.sleep(0.5)  # pause between moves
        self.get_logger().info("Workflow complete.")

def main():
    rclpy.init()
    node = PDMSWorkflow()
    time.sleep(1.0)  # let publisher connect
    node.run_workflow()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
