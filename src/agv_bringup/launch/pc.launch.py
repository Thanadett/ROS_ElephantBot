"""
pc.launch.py  — 660610822 Final Project
Launch PC-side nodes  (ROS2 Jazzy, ROS_DOMAIN_ID=1)

รัน:
    export ROS_DOMAIN_ID=1
    ros2 launch agv_bringup pc.launch.py robot_ip:=192.168.1.100

Flow:
  hand_controller → /hand/cmd_vel
  motion_manager  → /cmd_vel_command
  lidar_guard     ← /scan (จาก udp_gateway) + /cmd_vel_command → /cmd_vel_safe + /obstacle_info
  udp_gateway     ← /cmd_vel_safe → UDP:15000→Robot  |  Robot UDP:15001 → /scan
  dashboard_ui    ← /camera_image/compressed + /hand_control_state + /obstacle_info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_id',  default_value='0'),
        DeclareLaunchArgument('flip',       default_value='true'),
        DeclareLaunchArgument('max_lin',    default_value='0.50'),
        DeclareLaunchArgument('min_lin',    default_value='0.10'),
        DeclareLaunchArgument('max_ang',    default_value='1.00'),
        DeclareLaunchArgument('min_ang',    default_value='0.20'),
        DeclareLaunchArgument('robot_ip',   default_value='172.20.10.3',
                              description='Robot IP address'),
        DeclareLaunchArgument('cmd_port',   default_value='15000'),
        DeclareLaunchArgument('scan_port',  default_value='15001'),
    ]

    hand_controller_node = Node(
        package='agv_pc', executable='hand_controller',
        name='hand_controller', output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'flip':      LaunchConfiguration('flip'),
            'max_lin':   LaunchConfiguration('max_lin'),
            'min_lin':   LaunchConfiguration('min_lin'),
            'max_ang':   LaunchConfiguration('max_ang'),
            'min_ang':   LaunchConfiguration('min_ang'),
        }]
    )

    motion_manager_node = Node(
        package='agv_pc', executable='motion_manager',
        name='motion_manager', output='screen',
    )

    # lidar_guard บน PC: รับ /scan จาก udp_gateway กรองคำสั่ง → /cmd_vel_safe
    lidar_guard_node = Node(
        package='agv_pc', executable='lidar_guard',
        name='lidar_guard', output='screen',
    )

    # udp_gateway: /cmd_vel_safe→UDP:15000 + UDP:15001→/scan
    udp_gateway_node = Node(
        package='agv_pc', executable='udp_gateway',
        name='udp_gateway', output='screen',
        parameters=[{
            'robot_ip':  LaunchConfiguration('robot_ip'),
            'cmd_port':  LaunchConfiguration('cmd_port'),
            'scan_port': LaunchConfiguration('scan_port'),
        }]
    )

    dashboard_node = Node(
        package='agv_pc', executable='dashboard_ui',
        name='ui_node', output='screen',
    )

    return LaunchDescription([
        LogInfo(msg="═══ 660610822 AGV — PC Nodes (Jazzy, DOMAIN_ID=1) ═══"),
        LogInfo(msg="hand_controller | motion_manager | lidar_guard | udp_gateway | dashboard_ui"),
        *args,
        hand_controller_node,
        motion_manager_node,
        lidar_guard_node,
        udp_gateway_node,
        dashboard_node,
    ])