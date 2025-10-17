from setuptools import setup
import os
from glob import glob

package_name = 'safety_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 This line ensures all launch files are included
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorina',
    maintainer_email='jorina@todo.todo',
    description='Automatic Emergency Braking (AEB) node for F1TENTH simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_node = safety_node.aeb_node:main',
        ],
    },
)
