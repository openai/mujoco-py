#!/usr/bin/env python
import os
from mujoco_py.utils import discover_mujoco
from mujoco_py.mujoco_exceptions import ignore_mujoco_warnings, MujocoException, user_warning_raise_exception, find_activation_key
from mujoco_py.generated import const
from mujoco_py.mjrenderpool import MjRenderPool
from mujoco_py.version import __version__, get_version
from mujoco_py.load_cymj import load_cymj, get_functions_from_cymj


cymj = load_cymj()
functions = get_functions_from_cymj()

# mjviewer needs cymj. Therefore import can happen only after.
from mujoco_py.mjviewer import MjViewer, MjViewerBasic

load_model_from_path = cymj.load_model_from_path
load_model_from_xml = cymj.load_model_from_xml
load_model_from_mjb = cymj.load_model_from_mjb
MjSim = cymj.MjSim
MjSimState = cymj.MjSimState
MjRenderContext = cymj.MjRenderContext
MjRenderContextOffscreen = cymj.MjRenderContextOffscreen
MjRenderContextWindow = cymj.MjRenderContextWindow
MjBatchRenderer = cymj.MjBatchRenderer
GlfwContext = cymj.GlfwContext


# Public API:
__all__ = ['MjSim', 'MjSimState',
           'MjRenderContextOffscreen', 'MjRenderContextWindow',
           'MjRenderContext', 'MjViewer', 'MjViewerBasic',
           'MujocoException', 'MjRenderPool', 'MjBatchRenderer', 'GlfwContext',
           'load_model_from_path', 'load_model_from_xml',
           'load_model_from_mjb',
           'ignore_mujoco_warnings', 'const', "functions",
           "__version__", "get_version", "cymj"]


# Print out a warning if we can't find the key.
# this is nicer than failing activation, which we can not do in python.
# The mujoco library exits the process forcibly, in a way we can't try/catch.
find_activation_key()
if not os.environ.get('MUJOCO_PY_SKIP_ACTIVATE'):
    mujoco_path, key_path = discover_mujoco()
    functions.mj_activate(key_path)
