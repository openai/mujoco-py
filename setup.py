#!/usr/bin/env python

from distutils.core import setup

setup(
    name='mujoco-py',
    version='0.4.3',
    description='Python wrapper for Mojoco',
    author='OpenAI',
    packages=['mujoco_py'],
    install_requires=[
        'PyOpenGL>=3.1.0',
        'numpy>=1.10.4',
        'six',
    ],
    tests_requires=[
        'nose2'
    ]
)
