#!/usr/bin/env python
import os
from ctypes import cdll


MJ_BIN_PATH = '.mujoco/mjpro150/bin'.split('/')
cdll.LoadLibrary(os.path.join(os.getenv('HOME'), *MJ_BIN_PATH, 'libmujoco150.dylib'))
cdll.LoadLibrary(os.path.join(os.getenv('HOME'), *MJ_BIN_PATH, 'libglfw.3.dylib'))


import mujoco_py.cymj as cymj

from mujoco_py.builder import ignore_mujoco_warnings, functions, MujocoException
from mujoco_py.generated import const
from mujoco_py.mjrenderpool import MjRenderPool
from mujoco_py.mjviewer import MjViewer, MjViewerBasic
from mujoco_py.version import __version__, get_version
import mujoco_py

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
           "__version__", "get_version"]

mujoco_py.builder.activate()
