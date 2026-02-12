from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'traj_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Minimal waypoint publisher',
    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'waypoint_publisher = traj_planner.waypoint_publisher:main',
        ],
    },
)
