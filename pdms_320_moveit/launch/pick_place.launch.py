"""Collision-checked pick-and-place: MoveIt plans every motion.

    ros2 launch pdms_320_moveit pick_place.launch.py
    ros2 launch pdms_320_moveit pick_place.launch.py station:=1     # right syringe
    ros2 launch pdms_320_moveit pick_place.launch.py station:=2     # the mixer

Brings up move_group, the mock ros2_control system, the controllers, RViz, and
then the planner-driven sequence. Unlike workcell_sequence.py, every motion here
is checked against the SRDF collision matrix - the arm against itself and against
the frame, rail and holders - so a path that would clip the crossbar is refused
rather than executed.

    station:=0|1|2   which station to cycle   (default 0, left syringe)
    sequence:=false  bring up MoveIt only, plan by hand in RViz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("pdms_320_moveit")

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "demo.launch.py")))

    # The sequence needs move_group and the controllers up first. 15 s is
    # generous; the node also waits on the action server for up to 90 s.
    seq = TimerAction(period=15.0, actions=[ExecuteProcess(
        cmd=["python3", os.path.join(pkg, "tools", "moveit_pick_place.py"),
             "--ros-args", "-p", ["station:=", LaunchConfiguration("station")]],
        name="moveit_pick_place", output="screen",
        condition=IfCondition(LaunchConfiguration("sequence")))])

    return LaunchDescription([
        DeclareLaunchArgument("station", default_value="0",
                              description="0 = left syringe, 1 = right syringe, 2 = mixer"),
        DeclareLaunchArgument("sequence", default_value="true",
                              description="false = MoveIt only, plan by hand in RViz"),
        demo,      # demo.launch.py already starts pipette_attach
        seq,
    ])
