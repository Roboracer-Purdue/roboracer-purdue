from setuptools import setup
from glob import glob
import os

package_name = "f1tenth_race_day"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="arc",
    maintainer_email="arc@todo.com",
    description="Race day head-to-head F1TENTH controller using Frenet offsets, pure pursuit, LiDAR guardian, and recovery.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "race_day_node = f1tenth_race_day.race_day_node:main",
        ],
    },
)