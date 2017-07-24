#!/usr/bin/env python

import setuptools

setuptools.setup(
    name='cuda_buffer_ops',
    version='0.1',
    description='Tensorflow ops for reading CUDA buffers',
    maintainer='OpenAI',
    packages=['cuda_buffer_ops'],
    package_data={'cuda_buffer_ops': ['cuda_buffer_ops.so']},
    url='https://github.com/openai/mujoco-py')
