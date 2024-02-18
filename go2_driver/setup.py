#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


packages = ['go2_driver']

setup_args = generate_distutils_setup(
    packages=packages,
    package_dir={'': 'src'}
)

setup(**setup_args)
