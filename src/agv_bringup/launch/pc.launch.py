"""
pc.launch.py  — 660610822 Final Project
Launch PC-side nodes  (ROS2 Jazzy, ROS_DOMAIN_ID=1)

รัน:
    export ROS_DOMAIN_ID=1
    ros2 launch agv_bringup pc.launch.py robot_ip:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('flip',      default_value='true'),
        DeclareLaunchArgument('max_lin',   default_value='0.50'),
        DeclareLaunchArgument('min_lin',   default_value='0.10'),
        DeclareLaunchArgument('max_ang',   default_value='1.00'),
        DeclareLaunchArgument('min_ang',   default_value='0.20'),
        DeclareLaunchArgument('robot_ip',  default_value='127.0.0.1',
                              description='Robot IP address for UDP'),
        DeclareLaunchArgument('udp_port',  default_value='15000'),
    ]

    # ── hand_controller: กล้อง → /hand/cmd_vel + /hand_control_state + /camera_image ──
    hand_controller_node = Node(
        package='agv_pc',
        executable='hand_controller',
        name='hand_controller',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'flip':      LaunchConfiguration('flip'),
            'max_lin':   LaunchConfiguration('max_lin'),
            'min_lin':   LaunchConfiguration('min_lin'),
            'max_ang':   LaunchConfiguration('max_ang'),
            'min_ang':   LaunchConfiguration('min_ang'),
        }]
    )

    # ── motion_manager: gatekeeper + service /motion/set_override ──
    motion_manager_node = Node(
        package='agv_pc',
        executable='motion_manager',
        name='motion_manager',
        output='screen',
    )

    # ── udp_sender: /cmd_vel_command → UDP → Robot ────────────────
    udp_sender_node = Node(
        package='agv_pc',
        executable='udp_sender',
        name='udp_sender',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'udp_port': LaunchConfiguration('udp_port'),
        }]
    )

    # ── dashboard_ui: แสดงผล Dashboard (แทน hand_controller imshow) ──
    dashboard_node = Node(
        package='agv_pc',
        executable='dashboard_ui',
        name='ui_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg="═══ 660610822 AGV — PC Nodes (Jazzy, DOMAIN_ID=1) ═══"),
        LogInfo(msg="Nodes: hand_controller | motion_manager | udp_sender | dashboard_ui"),
        *args,
        hand_controller_node,
        motion_manager_node,
        udp_sender_node,
        dashboard_node,
    ])