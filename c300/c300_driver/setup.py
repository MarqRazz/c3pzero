# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup

package_name = "c300_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marq",
    maintainer_email="marq.razz@gmail.com",
    description="Driver package to control the c300 mobile base",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist2roboclaw = c300_driver.twist2roboclaw:main",
        ],
    },
)
