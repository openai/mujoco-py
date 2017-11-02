# mujoco-py [![Documentation](https://img.shields.io/badge/docs-latest-brightgreen.svg?style=flat)](https://openai.github.io/mujoco-py/build/html/index.html) [![Build Status](https://travis-ci.org/openai/mujoco-py.svg?branch=master)](https://travis-ci.org/openai/mujoco-py) [![Build status](https://ci.appveyor.com/api/projects/status/iw52c0198j87s76w?svg=true)](https://ci.appveyor.com/project/wojzaremba/mujoco-py)

[MuJoCo](http://mujoco.org/) is a physics engine for detailed, efficient rigid body simulations with contacts. `mujoco-py` allows using MuJoCo from Python 3.

## Synopsis

### Requirements

The following platforms are currently supported:

- Linux with Python 3.5.2. See [the `Dockerfile`](Dockerfile) for the canonical list of system dependencies. Support for Python 3.6 [is planned](https://github.com/openai/mujoco-py/issues/52).
- OS X with Python 3.5.2. Support for Python 3.6 [is planned](https://github.com/openai/mujoco-py/issues/52).
- Windows (experimental) with Python 3.5.2. See [the Appveyor file](https://github.com/openai/mujoco-py/blob/master/.appveyor.yml#L16-L32) for the canonical list of dependencies.

Python 2 has been desupported since [1.50.1.0](https://github.com/openai/mujoco-py/releases/tag/1.50.1.0). Python 2 users can stay on the [`0.5` branch](https://github.com/openai/mujoco-py/tree/0.5). The latest release there is [`0.5.7`](https://github.com/openai/mujoco-py/releases/tag/0.5.7) which can be installed with `pip install mujoco-py==0.5.7`.

### Install MuJoCo

1. Obtain a 30-day free trial on the [MuJoCo website](https://www.roboti.us/license.html)
   or free license if you are a student.
   The license key will arrive in an email with your username and password.
2. Download the MuJoCo version 1.50 binaries for
   [Linux](https://www.roboti.us/download/mjpro150_linux.zip),
   [OSX](https://www.roboti.us/download/mjpro150_osx.zip), or
   [Windows](https://www.roboti.us/download/mjpro150_windows.zip).
3. Unzip the downloaded `mjpro150` directory into `~/.mujoco/mjpro150`,
   and place your license key (the `mjkey.txt` file from your email)
   at `~/.mujoco/mjkey.txt`.

### Install and use `mujoco-py`
To include `mujoco-py` in your own package, add it to your requirements like so:
```
mujoco-py<1.50.2,>=1.50.1
```
To play with `mujoco-py` interactively, follow these steps:
```
$ pip3 install -U 'mujoco-py<1.50.2,>=1.50.1'
$ python3
import mujoco_py
from os.path import dirname
model = mujoco_py.load_model_from_path(dirname(dirname(mujoco_py.__file__))  +"/xmls/claw.xml")
sim = mujoco_py.MjSim(model)

print(sim.data.qpos)
# [ 0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.  0.]

sim.step()
print(sim.data.qpos)
# [  2.09217903e-06  -1.82329050e-12  -1.16711384e-07  -4.69613872e-11
#   -1.43931860e-05   4.73350204e-10  -3.23749942e-05  -1.19854057e-13
#   -2.39251380e-08  -4.46750545e-07   1.78771599e-09  -1.04232280e-08]
```

See the [full documentation](https://openai.github.io/mujoco-py/build/html/index.html) for advanced usage.

### Missing GLFW

A common error when installing is:

    raise ImportError("Failed to load GLFW3 shared library.")

Which happens when the `glfw` python package fails to find a GLFW dynamic library.

MuJoCo ships with its own copy of this library, which can be used during installation.

Add the path to the mujoco bin directory to your dynamic loader:

    LD_LIBRARY_PATH=$HOME/.mujoco/mjpro150/bin pip install mujoco-py

This is particularly useful on Ubuntu 14.04, which does not have a GLFW package.

## Usage Examples

A number of examples demonstrating some advanced features of `mujoco-py` can be found in [`examples/`](/./examples/). These include:
- [`body_interaction.py`](./examples/body_interaction.py): shows interactions between colliding bodies
- [`disco_fetch.py`](./examples/disco_fetch.py): shows how `TextureModder` can be used to randomize object textures
- [`internal_functions.py`](./examples/internal_functions.py): shows how to call raw mujoco functions like `mjv_room2model`
- [`markers_demo.py`](./examples/markers_demo.py): shows how to add visualization-only geoms to the viewer
- [`serialize_model.py`](./examples/serialize_model.py): shows how to save and restore a model
- [`setting_state.py`](./examples/setting_state.py):  shows how to reset the simulation to a given state
- [`simpool.py`](./examples/simpool.py): shows how `MjSimPool` can be used to run a number of simulations in parallel
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

## Credits

`mujoco-py` is maintained by the OpenAI Robotics team. Contributors include:

- Alex Ray
- Bob McGrew
- Jonas Schneider
- Jonathan Ho
- Peter Welinder
- Wojciech Zaremba
