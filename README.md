**Status:** Maintenance (expect bug fixes and minor updates)

# mujoco-py [![Documentation](https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat)](https://openai.github.io/mujoco-py/build/html/index.html) [![Build Status](https://travis-ci.org/openai/mujoco-py.svg?branch=master)](https://travis-ci.org/openai/mujoco-py)

[MuJoCo](http://mujoco.org/) is a physics engine for detailed, efficient rigid body simulations with contacts.
`mujoco-py` allows using MuJoCo from Python 3.

This library has been updated to be compatible with MuJoCo version 2.1 released on 2021-10-18.


## Synopsis

### Requirements

The following platforms are currently supported:

- Linux with Python 3.6+. See [the `Dockerfile`](Dockerfile) for the canonical list of system dependencies.
- OS X with Python 3.6+.

The following platforms are DEPRECATED and unsupported:

- Windows support has been DEPRECATED and removed in [2.0.2.0](https://github.com/openai/mujoco-py/releases/tag/v2.0.2.0a1). One known good past version is [1.50.1.68](https://github.com/openai/mujoco-py/blob/9ea9bb000d6b8551b99f9aa440862e0c7f7b4191/README.md#requirements).
- Python 2 has been DEPRECATED and removed in [1.50.1.0](https://github.com/openai/mujoco-py/releases/tag/1.50.1.0). Python 2 users can stay on the [`0.5` branch](https://github.com/openai/mujoco-py/tree/0.5). The latest release there is [`0.5.7`](https://github.com/openai/mujoco-py/releases/tag/0.5.7) which can be installed with `pip install mujoco-py==0.5.7`.

### Install MuJoCo

1. Download the MuJoCo version 2.1 binaries for
   [Linux](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz) or
   [OSX](https://mujoco.org/download/mujoco210-macos-x86_64.tar.gz).
1. Extract the downloaded `mujoco210` directory into `~/.mujoco/mujoco210`.

If you want to specify a nonstandard location for the package,
use the env variable `MUJOCO_PY_MUJOCO_PATH`.

### Install and use `mujoco-py`
To include `mujoco-py` in your own package, add it to your requirements like so:
```
mujoco-py<2.2,>=2.1
```
To play with `mujoco-py` interactively, follow these steps:
```
$ pip3 install -U 'mujoco-py<2.2,>=2.1'
$ python3
import mujoco_py
import os
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)

print(sim.data.qpos)
# [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]

sim.step()
print(sim.data.qpos)
# [-2.09531783e-19  2.72130735e-05  6.14480786e-22 -3.45474715e-06
#   7.42993721e-06 -1.40711141e-04 -3.04253586e-04 -2.07559344e-04
#   8.50646247e-05 -3.45474715e-06  7.42993721e-06 -1.40711141e-04
#  -3.04253586e-04 -2.07559344e-04 -8.50646247e-05  1.11317030e-04
#  -7.03465386e-05 -2.22862221e-05 -1.11317030e-04  7.03465386e-05
#  -2.22862221e-05]
```

See the [full documentation](https://openai.github.io/mujoco-py/build/html/index.html) for advanced usage.

## Troubleshooting

### You're on MacOS and you see `clang: error: unsupported option '-fopenmp'`

If this happend during installation or just running `python -c "import mujoco_py"` then the issue seems to be related to [this](https://github.com/velocyto-team/velocyto.R/issues/2#issuecomment-341165967) and the TL;DR is that for macOS the default compiler Apple clang LLVM does not support openmp. So you can try to install another clang/llvm installation. For example (requires [brew](https://brew.sh/)):

```bash
brew install llvm
brew install boost
brew install hdf5

# Add this to your .bashrc/.zshrc:
export PATH="/usr/local/opt/llvm/bin:$PATH"

export CC="/usr/local/opt/llvm/bin/clang"
export CXX="/usr/local/opt/llvm/bin/clang++"
export CXX11="/usr/local/opt/llvm/bin/clang++"
export CXX14="/usr/local/opt/llvm/bin/clang++"
export CXX17="/usr/local/opt/llvm/bin/clang++"
export CXX1X="/usr/local/opt/llvm/bin/clang++"

export LDFLAGS="-L/usr/local/opt/llvm/lib"
export CPPFLAGS="-I/usr/local/opt/llvm/include"
```

**Note:** Don't forget to source your `.bashrc/.zshrc` after editing it and try to install `mujoco-py` again:

```bash
# Make sure your python environment is activated
pip install -U 'mujoco-py<2.2,>=2.1'
```

### Missing GLFW

A common error when installing is:

    raise ImportError("Failed to load GLFW3 shared library.")

Which happens when the `glfw` python package fails to find a GLFW dynamic library.

MuJoCo ships with its own copy of this library, which can be used during installation.

Add the path to the mujoco bin directory to your dynamic loader:

    LD_LIBRARY_PATH=$HOME/.mujoco/mujoco210/bin pip install mujoco-py

This is particularly useful on Ubuntu 14.04, which does not have a GLFW package.


### Ubuntu installtion troubleshooting

Because `mujoco_py` has compiled native code that needs to be linked to a supplied MuJoCo binary, it's installation
on linux can be more challenging than pure Python source packages.

To install mujoco-py on Ubuntu, make sure you have the following libraries installed:

    sudo apt install libosmesa6-dev libgl1-mesa-glx libglfw3

If you installed above libraries and you still see an error that `-lGL` cannot be found, most likely you need
to create the symbolic link directly:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libGL.so.1 /usr/lib/x86_64-linux-gnu/libGL.so


## Usage Examples

A number of examples demonstrating some advanced features of `mujoco-py` can be found in [`examples/`](/./examples/). These include:
- [`body_interaction.py`](./examples/body_interaction.py): shows interactions between colliding bodies
- [`disco_fetch.py`](./examples/disco_fetch.py): shows how `TextureModder` can be used to randomize object textures
- [`internal_functions.py`](./examples/internal_functions.py): shows how to call raw mujoco functions like `mjv_room2model`
- [`markers_demo.py`](./examples/markers_demo.py): shows how to add visualization-only geoms to the viewer
- [`serialize_model.py`](./examples/serialize_model.py): shows how to save and restore a model
- [`setting_state.py`](./examples/setting_state.py):  shows how to reset the simulation to a given state
- [`tosser.py`](./examples/tosser.py): shows a simple actuated object sorting robot application

See the [full documentation](https://openai.github.io/mujoco-py/build/html/index.html) for advanced usage.

## Development

To run the provided unit and integrations tests:

```
make test
```

To test GPU-backed rendering, run:

```
make test_gpu
```

This is somewhat dependent on internal OpenAI infrastructure at the moment, but it should run if you change the `Makefile` parameters for your own setup.

## Changelog

- 03/08/2018: We removed MjSimPool, because most of benefit one can get with multiple processes having single simulation.

## Credits

`mujoco-py` is maintained by the OpenAI Robotics team. Contributors include:

- Alex Ray
- Bob McGrew
- Jonas Schneider
- Jonathan Ho
- Peter Welinder
- Wojciech Zaremba
- Jerry Tworek
