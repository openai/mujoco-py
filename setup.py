#!/usr/bin/env python3
import importlib.util
from distutils.command.build import build as DistutilsBuild
from os.path import abspath, join, dirname, realpath
from setuptools import find_packages, setup

with open(join("mujoco_py", "version.py")) as version_file:
    exec(version_file.read())


class Build(DistutilsBuild):
    def run(self):
        # Pre-compile the Cython
        current_path = abspath(dirname(__file__))
        builder_path = join(current_path, 'mujoco_py', 'builder.py')
        spec = importlib.util.spec_from_file_location(
            "mujoco_py.builder", builder_path)
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)

        DistutilsBuild.run(self)


def read_requirements_file(filename):
    req_file_path = '%s/%s' % (dirname(realpath(__file__)), filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]


packages = find_packages()
# Ensure that we don't pollute the global namespace.
for p in packages:
    assert p == 'mujoco_py' or p.startswith('mujoco_py.')

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
    install_requires=read_requirements_file('requirements.txt'),
    tests_require=read_requirements_file('requirements.dev.txt'),
    # Add requirements for mujoco_py/builder.py here since there's no
    # guarantee that they've been installed before this setup script
    # is run. (The install requirements only guarantee that those packages
    # are installed as part of installation. No promises about order.)
    setup_requires=read_requirements_file('requirements.txt'),
    cmdclass={'build': Build},
)
