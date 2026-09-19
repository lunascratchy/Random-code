from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auracle_slam_tuning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Offline SLAM parameter tuning harness (Optuna + evo + STL ground truth).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'record_waypoints = auracle_slam_tuning.record_waypoints:main',
            'course_driver = auracle_slam_tuning.course_driver:main',
            'gt_trajectory_logger = auracle_slam_tuning.gt_trajectory_logger:main',
        ],
    },
)
