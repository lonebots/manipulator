"""_
    summary_ setup file
"""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.[ux][ra][dc][fr]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[yr][va][mi][z]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.sdf"))
        )
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jishnu",
    maintainer_email="jishnusurajila@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
