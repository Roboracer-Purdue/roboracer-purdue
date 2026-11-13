from setuptools import find_packages, setup

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PatchyKawaiiDesu',
    maintainer_email='jsuantho@purdue.edu',
    description='Roborace @ Purdue Lab 2 Template',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follower = wall_follower.pid_template:main',
        ],
    },
)
