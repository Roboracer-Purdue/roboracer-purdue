from setuptools import setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wall_follow_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorina',
    maintainer_email='jorina@todo.todo',
    description='Wall following PID controller for F1TENTH car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow_node = wall_follow.wall_follow_node:main',
        ],
    },
)
