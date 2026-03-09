from setuptools import setup

package_name = 'agv_robot'

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
    description='Robot-side nodes for AGV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_guard    = agv_robot.lidar_guard:main',
            'udp_cmd_relay  = agv_robot.udp_cmd_relay:main',
        ],
    },
)
