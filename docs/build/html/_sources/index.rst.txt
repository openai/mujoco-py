=======================
mujoco-py Documentation
=======================

.. toctree::
   reference
   internals

`MuJoCo <http://mujoco.org/>`_ is a physics engine for detailed, efficient rigid body simulations with contacts. ``mujoco-py`` allows using MuJoCo from Python 3.

See the `README <https://github.com/openai/mujoco-py/blob/master/README.md>`_ for installation instructions and example usage.

``mujoco-py`` allows access to MuJoCo on a number of different levels of abstraction:

- Directly from Cython (low-level): `Raw Cython declarations <https://github.com/openai/mujoco-py/tree/master/mujoco_py/pxd>`_ are provided for using the MuJoCo C structs and functions directly in your own Cython code.

- Using :ref:`pymjdata` (medium-level): These wrappers are lightweight Cython ``cdef`` classes that expose MuJuCo data to Python space. The data in the MuJoCo structs is exposed as NumPy arrays bound to Mujoco-allocated memory, so there is no copying overhead when accessing or modifying MuJoCo state from Python. For more information on how this works internally, see [this document](./doc/cython_wrappers.md).

- Using :class:`mujoco_py.MjSim` (high-level): :class:`mujoco_py.MjSim` manages a stateful simulation similar to the `MujocoEnv <https://github.com/openai/gym/blob/master/gym/envs/mujoco/mujoco_env.py>`_ class found in `Gym <https://github.com/openai/gym>`_
.
