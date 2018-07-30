import os
import sys
from mujoco_py.utils import discover_mujoco


def _add_mujoco_bin_to_dyld_library_path():
    mujoco_path, _ = discover_mujoco()
    bin_path = os.path.join(mujoco_path, "bin")
    old_dyld_library_path = os.getenv("DYLD_LIBRARY_PATH", "")
    os.environ["DYLD_LIBRARY_PATH"] = "{}:{}".format(
        bin_path, old_dyld_library_path)


try:
    _add_mujoco_bin_to_dyld_library_path()
    import glfw
except ImportError:
    pass


class GlfwError(RuntimeError):
    pass


class GlfwContext():

    _INIT_WIDTH = 1000
    _INIT_HEIGHT = 1000
    _GLFW_IS_INITIALIZED = False

    def __init__(self, offscreen=False):
        GlfwContext._init_glfw()

        self._width = self._INIT_WIDTH
        self._height = self._INIT_HEIGHT
        self.window = self._create_window(offscreen)
        self._set_window_size(self._width, self._height)

    @staticmethod
    def _init_glfw():
        if GlfwContext._GLFW_IS_INITIALIZED:
            return

        if 'glfw' not in globals():
            raise GlfwError("GLFW not installed")

        glfw.set_error_callback(GlfwContext._glfw_error_callback)

        # HAX: sometimes first init() fails, while second works fine.
        glfw.init()
        if not glfw.init():
            raise GlfwError("Failed to initialize GLFW")

        GlfwContext._GLFW_IS_INITIALIZED = True

    def make_context_current(self):
        glfw.make_context_current(self.window)

    def set_buffer_size(self, width, height):
        self._set_window_size(width, height)
        self._width = width
        self._height = height

    def _create_window(self, offscreen):
        if offscreen:
            print("Creating offscreen glfw")
            glfw.window_hint(glfw.VISIBLE, 0)
            glfw.window_hint(glfw.DOUBLEBUFFER, 0)
            init_width, init_height = self._INIT_WIDTH, self._INIT_HEIGHT
        else:
            print("Creating window glfw")
            glfw.window_hint(glfw.SAMPLES, 4)
            glfw.window_hint(glfw.VISIBLE, 1)
            glfw.window_hint(glfw.DOUBLEBUFFER, 1)
            resolution, _, refresh_rate = glfw.get_video_mode(
                glfw.get_primary_monitor())
            init_width, init_height = resolution

        self._width = init_width
        self._height = init_height
        window = glfw.create_window(
            self._width, self._height, "mujoco_py", None, None)

        if not window:
            raise GlfwError("Failed to create GLFW window")

        return window

    def get_buffer_size(self):
        return glfw.get_framebuffer_size(self.window)

    def _set_window_size(self, target_width, target_height):
        self.make_context_current()
        if target_width != self._width or target_height != self._height:
            self._width = target_width
            self._height = target_height
            glfw.set_window_size(self.window, target_width, target_height)

            # HAX: When running on a Mac with retina screen, the size
            # sometimes doubles
            width, height = glfw.get_framebuffer_size(self.window)
            if target_width != width:
                glfw.set_window_size(self.window, target_width // 2, target_height // 2)

    @staticmethod
    def _glfw_error_callback(error_code, description):
        print("GLFW error (code %d): %s", error_code, description)


