import os
from setuptools import setup
from glob import glob


package_name = 'odrive_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jiang Yue',
    maintainer_email='maze1024@gmail.com',
    description='ROS2 node for ODrive',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = odrive_ros2.odrive_node:main',
            'odometry_node = odrive_ros2.odometry_node:main',
            'vel_node = odrive_ros2.vel_node:main',
            'odrive_config = odrive_ros2.odrive_config:main',
            'state0_config = odrive_ros2.state0_config:main',
            'state1_config = odrive_ros2.state1_config:main',
            'state0_con_config = odrive_ros2.state0_con_config:main',
            'state1_con_config = odrive_ros2.state1_con_config:main',
            'twist_to_motors = odrive_ros2.twist_to_motors:main'


        ],
    },
)
