from setuptools import setup

package_name = 'agv_pc'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t',
    maintainer_email='n.thanadett@gmail.com',
    description='PC-side nodes for AGV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hand_controller = agv_pc.hand_controller:main',
            'motion_manager  = agv_pc.motion_manager:main',
            'udp_gateway     = agv_pc.udp_gateway:main',
            'dashboard_ui   = agv_pc.dashboard_ui:main',
            'lidar_guard    = agv_pc.lidar_guard:main',
        ],
    },
)
