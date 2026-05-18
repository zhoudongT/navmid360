from glob import glob

from setuptools import find_packages, setup

package_name = "dong_dwa_local_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="zhangwanjie@126.com",
    description="DWA local planner extracted from MoveBase3D, ROS2 bridge for dong_3d_nav",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dwa_local_planner_node = dong_dwa_local_planner.dwa_local_planner_node:main",
        ],
    },
)
