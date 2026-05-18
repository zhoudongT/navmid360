from glob import glob

from setuptools import find_packages, setup

package_name = "dong_relocalization"

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
    description="PCD prior + live cloud ICP relocalization for dong_3d_nav",
    license="MIT",
    entry_points={
        "console_scripts": [
            "icp_relocalizer_node = dong_relocalization.icp_relocalizer_node:main",
        ],
    },
)
