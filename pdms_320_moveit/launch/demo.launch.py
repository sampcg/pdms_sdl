"""MoveIt demo: 320 + fork gripper + PDMS workcell.

    ros2 launch pdms_320_moveit demo.launch.py

Starts move_group, a mock ros2_control system, the controllers, RViz with the
MotionPlanning panel and robot_state_publisher. Plan with the interactive
marker, or send goals to the `arm_group` / `gripper` planning groups.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("firefighter", package_name="pdms_320_moveit")
        .to_moveit_configs()
    )
    ld = generate_demo_launch(moveit_config)

    # The pipette is an RViz Marker published by pipette_attach.py, not a URDF
    # link, so the MoveIt demo needs it started explicitly or nothing shows up.
    desc = get_package_share_directory("pdms_gripper_description")
    ld.add_action(ExecuteProcess(
        cmd=["python3", os.path.join(desc, "tools", "pipette_attach.py")],
        name="pipette_attach",
        output="screen",
    ))
    return ld
