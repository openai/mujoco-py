#!/usr/bin/env python3
from setuptools import setup

setup(
    name='pymj',
    version='2.0',
    description='DEPRECATED -- use mujoco-py instead: https://github.com/openai/mujoco-py',
    url='https://github.com/openai/mujoco-py',
    author='OpenAI Robotics Team',
    author_email='robotics@openai.com',
    packages=['pymj'],
    install_requires=['mujoco-py<1.50.1,>=1.50.0'],
)
