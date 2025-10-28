from setuptools import setup

package_name = 'follow_gap'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='you@example.com',
    description='F1TENTH Follow the Gap algorithm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follow_gap = follow_gap.follow_gap:main',
        ],
    },
)
