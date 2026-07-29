"""Display the myCobot 320 Pi with the custom fork fingers in RViz.

No hardware needed - robot_state_publisher + joint_state_publisher_gui + RViz.
Drag the `gripper_controller` slider to open/close the forks.

    ros2 launch pdms_gripper_description fork_gripper_display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("pdms_gripper_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg, "urdf", "mycobot_320_pi_2022_fork_gripper.urdf"),
        description="URDF to display",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(pkg, "config", "fork_gripper.rviz"),
        description="RViz config",
    )
    gui_arg = DeclareLaunchArgument(
        name="gui", default_value="true",
        description="true = joint_state_publisher_gui sliders",
    )

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
        Node(
            name="rviz2",
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        ),
    ])
