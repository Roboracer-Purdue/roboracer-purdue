from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_pure_pursuit.py']),
        ('share/' + package_name + '/launch', ['launch/launch_pp_opp_car.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='patchy',
    maintainer_email='00441@kvis.ac.th',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit.pure_pursuit:main'
        ],
    },
)
