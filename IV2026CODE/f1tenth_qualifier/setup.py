from setuptools import setup

package_name = 'f1tenth_qualifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/qualifier.yaml']),
        ('share/' + package_name + '/launch', ['launch/qualifier.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arc',
    maintainer_email='arc@example.com',
    description='F1TENTH race-day qualifier algorithm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qualifier_node = f1tenth_qualifier.qualifier_node:main',
        ],
    },
)