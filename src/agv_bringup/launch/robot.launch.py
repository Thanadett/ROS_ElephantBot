"""
robot.launch.py  — 660610822 Final Project
Launch robot-side nodes  (ROS2 Galactic, ROS_DOMAIN_ID=0)

รัน: export ROS_DOMAIN_ID=0 && ros2 launch agv_bringup robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    udp_relay_node = Node(
        package='agv_robot',
        executable='udp_cmd_relay',
        name='udp_cmd_relay',
        output='screen',
    )

    lidar_guard_node = Node(
        package='agv_robot',
        executable='lidar_guard',
        name='lidar_guard',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg="═══ 660610822 AGV — Robot Nodes ═══"),
        LogInfo(msg="ROS2 Galactic  |  DOMAIN_ID=0"),
        LogInfo(msg="UDP:15000 ← PC(Jazzy)  |  /scan ← ros1_bridge(Noetic)"),
        LogInfo(msg="Topics: /cmd_vel_input → lidar_guard → /cmd_vel → ros1_bridge → Noetic"),
        udp_relay_node,
        lidar_guard_node,
    ])