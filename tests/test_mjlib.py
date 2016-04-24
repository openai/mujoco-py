"""
Test mujoco function bindings in mjlib.
This includes basic tests to make sure the function calls work.
"""
import ctypes
import unittest

from mujoco_py.mjlib import mjlib
from mujoco_py import mjtypes
from mujoco_py import glfw


class MjLibTest(unittest.TestCase):
    xml_path = 'tests/models/cartpole.xml'

    def setUp(self):
        self.buf = ctypes.create_string_buffer(300)
        self.model_ptr = mjlib.mj_loadXML(self.xml_path, None, self.buf, 300)
        self.data_ptr = mjlib.mj_makeData(self.model_ptr)

    def tearDown(self):
        self.buf = None
        mjlib.mj_deleteData(self.data_ptr)
        mjlib.mj_deleteModel(self.model_ptr)

    def test_model(self):
        self.assertEqual(len(self.buf.value), 0)

        self.assertIsInstance(self.model_ptr, ctypes.POINTER(mjtypes.MJMODEL))

    def test_data(self):
        self.assertIsNotNone(self.data_ptr)

        self.assertIsInstance(self.data_ptr, ctypes.POINTER(mjtypes.MJDATA))

    def test_step(self):
        start_time = self.data_ptr.contents.time
        mjlib.mj_step(self.model_ptr, self.data_ptr)
        self.assertGreater(self.data_ptr.contents.time, start_time)

    def test_forward(self):
        start_moment = self.data_ptr.contents.actuator_moment.contents.value
        mjlib.mj_forward(self.model_ptr, self.data_ptr)
        self.assertGreater(self.data_ptr.contents.actuator_moment.contents.value,
                           start_moment)

    def test_make_objects(self):
        objects = mjtypes.MJVOBJECTS()
        mjlib.mjv_makeObjects(ctypes.byref(objects), 1000)
        mjlib.mjv_freeObjects(ctypes.byref(objects))

    def test_default_camera(self):
        cam = mjtypes.MJVCAMERA()
        mjlib.mjv_defaultCamera(ctypes.byref(cam))

    def test_default_option(self):
        vopt = mjtypes.MJVOPTION()
        ropt = mjtypes.MJROPTION()
        mjlib.mjv_defaultOption(ctypes.byref(vopt))
        mjlib.mjr_defaultOption(ctypes.byref(ropt))

    def test_default_context(self):
        context = mjtypes.MJRCONTEXT()
        mjlib.mjr_defaultContext(ctypes.byref(context))

    def test_context(self):
        if not glfw.init():
            raise Exception('glfw failed to initialize')

        window = None
        glfw.window_hint(glfw.VISIBLE, 0)

        _, _, refresh_rate = glfw.get_video_mode(glfw.get_primary_monitor())
        if refresh_rate >= 100:
            glfw.window_hint(glfw.STEREO, 1)
            window = glfw.create_window(
                100, 100, "Simulate", None, None)

        # no stereo: try mono
        if not window:
            glfw.window_hint(glfw.STEREO, 0)
            window = glfw.create_window(
                100, 100, "Simulate", None, None)

        # Make the window's context current
        glfw.make_context_current(window)

        context = mjtypes.MJRCONTEXT()
        mjlib.mjr_makeContext(self.model_ptr, ctypes.byref(context), 150)

        mjlib.mjr_freeContext(ctypes.byref(context))
