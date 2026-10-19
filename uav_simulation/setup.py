from setuptools import setup
import os
from glob import glob

package_name = 'uav_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models', 'quadrotor'), glob('models/quadrotor/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'matplotlib'],
    zip_safe=True,
    maintainer='UAV Simulation Lab',
    maintainer_email='uav_sim@todo.todo',
    description='ROS 2 and Gazebo simulation of Quaternion-Based Attitude Tracking Control Design for UAVs (CACS 2024)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = uav_simulation.waypoint_manager:main',
            'trajectory_node = uav_simulation.trajectory_node:main',
            'controller_node = uav_simulation.controller_node:main',
            'gazebo_uav_node = uav_simulation.gazebo_uav_node:main',
            'info_panel_node = uav_simulation.info_panel_node:main',
            'data_logger = uav_simulation.data_logger:main',
        ],
    },
)
