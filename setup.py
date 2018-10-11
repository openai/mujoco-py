#!/usr/bin/env python3
from os.path import join, dirname, realpath
from os import getenv
from setuptools import find_packages, setup
from setuptools.extension import Extension
from Cython.Build import cythonize
from ctypes import cdll
import numpy as np

MJ_PATH = join(getenv('HOME'), '.mujoco', 'mjpro150')

# XXX this is mac specific for now
cdll.LoadLibrary(join(MJ_PATH, 'bin', 'libmujoco150.dylib'))
cdll.LoadLibrary(join(MJ_PATH, 'bin', 'libglfw.3.dylib'))


with open(join("mujoco_py", "version.py")) as version_file:
    exec(version_file.read())


def read_requirements_file(filename):
    req_file_path = '%s/%s' % (dirname(realpath(__file__)), filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]


packages = find_packages()
# Ensure that we don't pollute the global namespace.
for p in packages:
    assert p == 'mujoco_py' or p.startswith('mujoco_py.')


cymjpyx = join('mujoco_py', 'cymj.pyx')
include = join(MJ_PATH, 'include')
mjbin = join(MJ_PATH, 'bin')
print('include', include)


cymj_extension = Extension(name='mujoco_py.cymj',
                           sources=['mujoco_py/cymj.pyx'],
                           include_dirs=[include, np.get_include()],
                           library_dirs=[mjbin],
                           libraries=['mujoco150', 'glfw.3'])


ext_modules = [cymj_extension]


setup(
    name='mujoco-py',
    version=__version__,  # noqa
    author='OpenAI Robotics Team',
    author_email='robotics@openai.com',
    url='https://github.com/openai/mujoco-py',
    packages=packages,
    include_package_data=True,
    package_dir={'mujoco_py': 'mujoco_py'},
    package_data={'mujoco_py': ['generated/*.so']},
    ext_modules=cythonize(ext_modules),
    install_requires=read_requirements_file('requirements.txt'),
    tests_require=read_requirements_file('requirements.dev.txt'),
)
