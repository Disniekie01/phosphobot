from setuptools import setup
import os
from glob import glob

package_name = 'phospho_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets', 'numpy', 'requests'],
    zip_safe=True,
    maintainer='Disniekie',
    maintainer_email='disniekie@irlrobotics.dev',
    description='IRL Robotics teleoperation interface for SO-ARM101 robot via ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'phospho_teleop_node = phospho_teleop.phospho_teleop_node:main',
            'phospho_leader_controller = phospho_teleop.phospho_leader_controller:main',
            'phospho_status_monitor = phospho_teleop.phospho_status_monitor:main',
            'phospho_http_teleop = phospho_teleop.phospho_http_teleop:main',
        ],
    },
)
