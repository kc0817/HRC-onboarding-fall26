"""Bring up the whole sim2sim stack: simulator + policy (+ optional teleop).

    ros2 launch pup_bringup sim2sim.launch.py headless:=false teleop:=true
    ros2 launch pup_bringup sim2sim.launch.py policy_path:=/ws/runs/colab/policy.npz

`pup_sim/launch/sim_only.launch.py` is the template this is built from.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pup_bringup.policy_node import DEFAULT_POLICY_PATH


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the full sim2sim stack."""
    headless = LaunchConfiguration("headless")
    policy_path = LaunchConfiguration("policy_path")
    teleop = LaunchConfiguration("teleop")
    realtime_factor = LaunchConfiguration("realtime_factor")

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("policy_path", default_value=""),
            DeclareLaunchArgument("teleop", default_value="false"),
            DeclareLaunchArgument("realtime_factor", default_value="1.0"),
            Node(
                package="pup_bringup",
                executable="policy_node",
                name="pup_policy",
                output="screen",
                parameters=[{"policy_path": DEFAULT_POLICY_PATH}],
            ),
            Node(
                package="pup_sim",
                executable="teleop_node",
                name="pup_teleop",
                output="screen",
                condition=IfCondition(teleop),
            ),
            Node(
                package="pup_sim",
                executable="sim_node",
                name="pup_sim",
                output="screen",
                parameters=[{"headless": headless, "realtime_factor": realtime_factor}],
            ),
        ]
    )
