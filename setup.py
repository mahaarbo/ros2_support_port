#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import io
import os

from setuptools import find_packages, setup

# Package Metadata
NAME = "ros2_support_port"
DESCRIPTION = "Module for porting ROS Industrial robot support packages to ROS 2."
URL = "http://github.com/mahaarbo/ros2_support_port"
EMAIL = "mathias.arbo@ntnu.no"
AUTHOR = "Mathias Hauan Arbo"
REQUIRES_PYTHON = ">=3.5.0"
VERSION = "0.1.0"

REQUIRED = [
    ""
]

here = os.path.abspath(os.path.dirname(__file__))

with io.open(os.path.join(here, "README.md"), encoding="utf-8") as f:
    long_description = "\n" + f.read()

about = {}
if not VERSION:
    with open(os.path.join(here, NAME, "__version__.py")) as f:
        exec(f.read(), about)
else:
    about["__version__"] = VERSION


setup(
    name=NAME,
    version=about["__version__"],
    description=DESCRIPTION,
    long_description=long_description,
    author=AUTHOR,
    author_email=EMAIL,
    url=URL,
    packages=find_packages(),
    keywords=["ROS", "ROS2"],
    entry_points={
        "console_scripts": [
            "migrate_rosi_support = ros2_support_port.migrator:main"
        ],
    },
    install_requires=REQUIRED,
    python_requires=">3.6",
    include_package_data=True,
    license="MIT",
    classifiers=[
        # Trove classifiers
        # Full list: https://pypi.python.org/pypi?%3Aaction=list_classifiers
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python"
    ],
)