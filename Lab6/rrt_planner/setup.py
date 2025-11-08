from setuptools import setup
import os
from glob import glob

package_name = 'rrt_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Orina',
    maintainer_email='john@purdue.edu',
    description='Local RRT motion planner for F1TENTH vehicle.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_planner = rrt_planner.rrt_planner_node:main',
        ],
    },
)
