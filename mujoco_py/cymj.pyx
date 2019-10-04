# cython: language_level=3
import copy
import logging
import os
import platform
import tempfile
import sys
from collections import namedtuple
from libc.stdlib cimport malloc, free
from libc.string cimport strncpy
from numbers import Number
from tempfile import TemporaryDirectory

import numpy as np
from cython cimport view
from cython.parallel import parallel, prange
from mujoco_py.generated import const

include "generated/wrappers.pxi"
include "opengl_context.pyx"
include "mjsim.pyx"
include "mjsimstate.pyx"
include "mjrendercontext.pyx"
include "mjbatchrenderer.pyx"
include "mjpid.pyx"

cdef extern from "gl/glshim.h":

    cdef int usingEGL()
    cdef int initOpenGL(int device_id)
    cdef void closeOpenGL()
    cdef int makeOpenGLContextCurrent(int device_id)
    cdef int setOpenGLBufferSize(int device_id, int width, int height)

    cdef unsigned int createPBO(int width, int height, int batchSize, int use_short)
    cdef void freePBO(unsigned int pixelBuffer)
    cdef void copyFBOToPBO(mjrContext* con,
                           unsigned int pbo_rgb, unsigned int pbo_depth,
                           mjrRect viewport, int bufferOffset)
    cdef void readPBO(unsigned char *buffer_rgb, unsigned short *buffer_depth,
                      unsigned int pbo_rgb, unsigned int pbo_depth,
                      int width, int height, int batchSize)



# TODO: make this function or class so these comments turn into doc strings:

# Python warning callback function, which is set
# MuJoCo has a user-settable callback function for warnings: mju_user_warning()
# We want to supply a python function, and be able to raise exceptions.
# To do this we have to wrap two things:

# This is the python callback function.  We save it in the global() context
# so we can access it from a C wrapper function (c_warning_callback)
cdef object py_warning_callback
cdef object py_error_callback
# This is the saved exception.  Because the C callback can not propagate
# exceptions, this must be set to None before calling into MuJoCo, and then
# inspected afterwards.
# These are combined in a simple class which handles both:
#   with wrap_mujoco_warning():
#       mj_somefunc()
cdef object py_warning_exception = None
cdef object py_error_exception = None


cdef void c_warning_callback(const char *msg) with gil:
    '''
    Wraps the warning callback so we can raise exceptions.
    Because callbacks can't propagate exceptions, we set a global that has
        to be inspected later.
    Use wrap_mujoco_warning() to check for that saved exception and
        re-raise it back to the caller.
    '''
    global py_warning_callback
    try:
        (<object> py_warning_callback)(msg)
    except Exception as e:
        global py_warning_exception
        py_warning_exception = e


def set_warning_callback(warn):
    '''
    Set a user-defined warning callback.  It should take in a string message
        (the warning string) and raise an Exception.
    See c_warning_callback, which is the C wrapper to the user defined function
    '''
    global py_warning_callback
    global mju_user_warning
    py_warning_callback = warn
    mju_user_warning = c_warning_callback


def get_warning_callback():
    '''
    Returns the user-defined warning callback, for use in e.g. a context
    manager.
    '''
    global py_warning_callback
    return py_warning_callback


cdef void c_error_callback(const char *msg) with gil:
    '''
    Wraps the error callback so that we can pass a python function to the callback.
    MuJoCo error handlers are expected to terminate the program and never return.
    '''
    global py_error_callback

    try:
        (<object> py_error_callback)(msg)
    except Exception as e:
        global py_error_exception
        py_error_exception = e


def set_error_callback(err_callback):
    '''
    Set a user-defined error callback.  It should take in a string message
        (the warning string) and terminate the program.
    See c_warning_callback, which is the C wrapper to the user defined function
    '''
    global py_error_callback
    global mju_user_error
    py_error_callback = err_callback
    mju_user_error = c_error_callback


def get_error_callback():
    '''
    Returns the user-defined warning callback, for use in e.g. a context
    manager.
    '''
    global py_error_callback
    return py_error_callback


class wrap_mujoco_warning(object):
    '''
    Class to wrap capturing exceptions raised during warning callbacks.
    Use this to capture warnings in mujoco calls.  Example:
        with wrap_mujoco_warning():
            mj_somefunc()
    '''
    def __enter__(self):
        global py_warning_exception
        py_warning_exception = None
        global py_error_exception
        py_error_exception = None
    def __exit__(self, type, value, traceback):
        global py_warning_exception
        global py_error_exception

        if py_warning_exception is not None:
            raise py_warning_exception

        if py_error_exception is not None:
            raise py_error_exception


def load_model_from_path(str path):
    """Loads model from path."""
    cdef char errstr[300]
    cdef mjModel *model
    with wrap_mujoco_warning():
        if (path.endswith(".mjb")):
            model = mj_loadModel(path.encode(), NULL)
        elif (path.endswith(".xml")):
            model = mj_loadXML(path.encode(), NULL, errstr, 300)
        else:
            raise RuntimeError("Unrecognized extension for %s. Expected .xml or .mjb" % path)

    if model == NULL:
        raise Exception('Failed to load XML file: %s. mj_loadXML error: %s' % (path, errstr,))
    return WrapMjModel(model)

def load_model_from_xml(str xml_str):
    """
    Loads and returns a PyMjModel model from a string containing XML markup.
    Saves the XML string used to create the returned model in `model.xml`.
    """
    cdef char errstr[300]
    cdef mjModel *model
    with wrap_mujoco_warning():
        with tempfile.NamedTemporaryFile(suffix=".xml", delete=True) as fp:
            fp.write(xml_str.encode())
            fp.flush()
            model = mj_loadXML(fp.name.encode(), NULL, errstr, 300)
    if model == NULL:
        raise Exception('%s\nFailed to load XML from string. mj_loadXML error: %s' % (xml_str, errstr,))
    return WrapMjModel(model)


def load_model_from_mjb(bytes mjb_bytes):
    """
    Loads and returns a PyMjModel model from bytes encoded MJB.
    MJB is a MuJoCo-custom format that includes assets like meshes/textures.
    """
    cdef mjModel *model
    with wrap_mujoco_warning():
        with TemporaryDirectory() as td:
            filename = os.path.join(td, 'model.mjb')
            with open(filename, 'wb') as f:
                f.write(mjb_bytes)
            model = mj_loadModel(filename.encode(), NULL)
    if model == NULL:
        raise Exception('%s\nFailed to load MJB')
    return WrapMjModel(model)
