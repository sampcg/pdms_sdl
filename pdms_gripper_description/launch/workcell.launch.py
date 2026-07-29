"""Full workcell: 320 + fork gripper + 2020 frame + pipette holder + pipette.

    ros2 launch pdms_gripper_description workcell.launch.py

The pipette starts in the holder bore. Grasp / release it with:

    ros2 topic pub --once /pipette/attach std_msgs/Bool "{data: true}"
    ros2 topic pub --once /pipette/attach std_msgs/Bool "{data: false}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("pdms_gripper_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg, "urdf", "mycobot_320_pi_2022_workcell.urdf"),
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(pkg, "config", "workcell.rviz"),
    )
    gui_arg = DeclareLaunchArgument(name="gui", default_value="true")

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        gui_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(pkg, "tools", "pipette_attach.py")],
            name="pipette_attach",
            output="screen",
        ),
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        ),
    ])
