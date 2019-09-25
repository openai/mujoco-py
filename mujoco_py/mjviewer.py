import copy
import glfw
import imageio
import numpy as np
import time
import sys

from mujoco_py.builder import cymj
from mujoco_py.generated import const
from mujoco_py.utils import rec_copy, rec_assign
from multiprocessing import Process, Queue
from threading import Lock


class MjViewerBasic(cymj.MjRenderContextWindow):
    """
    A simple display GUI showing the scene of an :class:`.MjSim` with a mouse-movable camera.

    :class:`.MjViewer` extends this class to provide more sophisticated playback and interaction controls.

    Parameters
    ----------
    sim : :class:`.MjSim`
        The simulator to display.
    """

    def __init__(self, sim):
        super().__init__(sim)

        self._gui_lock = Lock()
        self._button_left_pressed = False
        self._button_right_pressed = False
        self._last_mouse_x = 0
        self._last_mouse_y = 0

        framebuffer_width, _ = glfw.get_framebuffer_size(self.window)
        window_width, _ = glfw.get_window_size(self.window)
        self._scale = framebuffer_width * 1.0 / window_width

        glfw.set_cursor_pos_callback(self.window, self._cursor_pos_callback)
        glfw.set_mouse_button_callback(
            self.window, self._mouse_button_callback)
        glfw.set_scroll_callback(self.window, self._scroll_callback)
        glfw.set_key_callback(self.window, self.key_callback)

    def render(self):
        """
        Render the current simulation state to the screen or off-screen buffer.
        Call this in your main loop.
        """
        if self.window is None:
            return
        elif glfw.window_should_close(self.window):
            glfw.terminate()
            sys.exit(0)

        with self._gui_lock:
            super().render()

        glfw.poll_events()

    def key_callback(self, window, key, scancode, action, mods):
        if action == glfw.RELEASE and key == glfw.KEY_ESCAPE:
            print("Pressed ESC")
            print("Quitting.")
            glfw.terminate()
            sys.exit(0)

    def _cursor_pos_callback(self, window, xpos, ypos):
        if not (self._button_left_pressed or self._button_right_pressed):
            return

        # Determine whether to move, zoom or rotate view
        mod_shift = (
            glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
            glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        if self._button_right_pressed:
            action = const.MOUSE_MOVE_H if mod_shift else const.MOUSE_MOVE_V
        elif self._button_left_pressed:
            action = const.MOUSE_ROTATE_H if mod_shift else const.MOUSE_ROTATE_V
        else:
            action = const.MOUSE_ZOOM

        # Determine
        dx = int(self._scale * xpos) - self._last_mouse_x
        dy = int(self._scale * ypos) - self._last_mouse_y
        width, height = glfw.get_framebuffer_size(window)

        with self._gui_lock:
            self.move_camera(action, dx / height, dy / height)

        self._last_mouse_x = int(self._scale * xpos)
        self._last_mouse_y = int(self._scale * ypos)

    def _mouse_button_callback(self, window, button, act, mods):
        self._button_left_pressed = (
            glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self._button_right_pressed = (
            glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        x, y = glfw.get_cursor_pos(window)
        self._last_mouse_x = int(self._scale * x)
        self._last_mouse_y = int(self._scale * y)

    def _scroll_callback(self, window, x_offset, y_offset):
        with self._gui_lock:
            self.move_camera(const.MOUSE_ZOOM, 0, -0.05 * y_offset)


class MjViewer(MjViewerBasic):
    """
    Extends :class:`.MjViewerBasic` to add video recording, interactive time and interaction controls.

    The key bindings are as follows:

    - TAB: Switch between MuJoCo cameras.
    - H: Toggle hiding all GUI components.
    - SPACE: Pause/unpause the simulation.
    - RIGHT: Advance simulation by one step.
    - V: Start/stop video recording.
    - T: Capture screenshot.
    - I: Drop into ``ipdb`` debugger.
    - S/F: Decrease/Increase simulation playback speed.
    - C: Toggle visualization of contact forces (off by default).
    - D: Enable/disable frame skipping when rendering lags behind real time.
    - R: Toggle transparency of geoms.
    - M: Toggle display of mocap bodies.
    - 0-4: Toggle display of geomgroups

    Parameters
    ----------
    sim : :class:`.MjSim`
        The simulator to display.
    """

    def __init__(self, sim):
        super().__init__(sim)

        self._ncam = sim.model.ncam
        self._paused = False  # is viewer paused.
        # should we advance viewer just by one step.
        self._advance_by_one_step = False

        # Vars for recording video
        self._record_video = False
        self._video_queue = Queue()
        self._video_idx = 0
        self._video_path = "/tmp/video_%07d.mp4"

        # vars for capturing screen
        self._image_idx = 0
        self._image_path = "/tmp/frame_%07d.png"

        # run_speed = x1, means running real time, x2 means fast-forward times
        # two.
        self._run_speed = 1.0
        self._loop_count = 0
        self._render_every_frame = False

        self._show_mocap = True  # Show / hide mocap bodies.
        self._transparent = False  # Make everything transparent.

        # this variable is estamated as a running average.
        self._time_per_render = 1 / 60.0
        self._hide_overlay = False  # hide the entire overlay.
        self._user_overlay = {}

    def render(self):
        """
        Render the current simulation state to the screen or off-screen buffer.
        Call this in your main loop.
        """

        def render_inner_loop(self):
            render_start = time.time()

            self._overlay.clear()
            if not self._hide_overlay:
                for k, v in self._user_overlay.items():
                    self._overlay[k] = copy.deepcopy(v)
                self._create_full_overlay()
            super().render()
            if self._record_video:
                frame = self._read_pixels_as_in_window()
                self._video_queue.put(frame)
            else:
                self._time_per_render = 0.9 * self._time_per_render + \
                    0.1 * (time.time() - render_start)

        self._user_overlay = copy.deepcopy(self._overlay)
        # Render the same frame if paused.
        if self._paused:
            while self._paused:
                render_inner_loop(self)
                if self._advance_by_one_step:
                    self._advance_by_one_step = False
                    break
        else:
            # inner_loop runs "_loop_count" times in expectation (where "_loop_count" is a float).
            # Therefore, frames are displayed in the real-time.
            self._loop_count += self.sim.model.opt.timestep * self.sim.nsubsteps / \
                (self._time_per_render * self._run_speed)
            if self._render_every_frame:
                self._loop_count = 1
            while self._loop_count > 0:
                render_inner_loop(self)
                self._loop_count -= 1
        # Markers and overlay are regenerated in every pass.
        self._markers[:] = []
        self._overlay.clear()

    def _read_pixels_as_in_window(self, resolution=None):
        # Reads pixels with markers and overlay from the same camera as screen.
        if resolution is None:
            resolution = glfw.get_framebuffer_size(self.sim._render_context_window.window)

        resolution = np.array(resolution)
        resolution = resolution * min(1000 / np.min(resolution), 1)
        resolution = resolution.astype(np.int32)
        resolution -= resolution % 16
        if self.sim._render_context_offscreen is None:
            self.sim.render(resolution[0], resolution[1])
        offscreen_ctx = self.sim._render_context_offscreen
        window_ctx = self.sim._render_context_window
        # Save markers and overlay from offscreen.
        saved = [copy.deepcopy(offscreen_ctx._markers),
                 copy.deepcopy(offscreen_ctx._overlay),
                 rec_copy(offscreen_ctx.cam)]
        # Copy markers and overlay from window.
        offscreen_ctx._markers[:] = window_ctx._markers[:]
        offscreen_ctx._overlay.clear()
        offscreen_ctx._overlay.update(window_ctx._overlay)
        rec_assign(offscreen_ctx.cam, rec_copy(window_ctx.cam))

        img = self.sim.render(*resolution)
        img = img[::-1, :, :] # Rendered images are upside-down.
        # Restore markers and overlay to offscreen.
        offscreen_ctx._markers[:] = saved[0][:]
        offscreen_ctx._overlay.clear()
        offscreen_ctx._overlay.update(saved[1])
        rec_assign(offscreen_ctx.cam, saved[2])
        return img

    def _create_full_overlay(self):
        if self._render_every_frame:
            self.add_overlay(const.GRID_TOPLEFT, "", "")
        else:
            self.add_overlay(const.GRID_TOPLEFT, "Run speed = %.3f x real time" %
                             self._run_speed, "[S]lower, [F]aster")
        self.add_overlay(
            const.GRID_TOPLEFT, "Ren[d]er every frame", "Off" if self._render_every_frame else "On")
        self.add_overlay(const.GRID_TOPLEFT, "Switch camera (#cams = %d)" % (self._ncam + 1),
                                             "[Tab] (camera ID = %d)" % self.cam.fixedcamid)
        self.add_overlay(const.GRID_TOPLEFT, "[C]ontact forces", "Off" if self.vopt.flags[
                         10] == 1 else "On")
        self.add_overlay(
            const.GRID_TOPLEFT, "Referenc[e] frames", "Off" if self.vopt.frame == 1 else "On")
        self.add_overlay(const.GRID_TOPLEFT,
                         "T[r]ansparent", "On" if self._transparent else "Off")
        self.add_overlay(
            const.GRID_TOPLEFT, "Display [M]ocap bodies", "On" if self._show_mocap else "Off")
        if self._paused is not None:
            if not self._paused:
                self.add_overlay(const.GRID_TOPLEFT, "Stop", "[Space]")
            else:
                self.add_overlay(const.GRID_TOPLEFT, "Start", "[Space]")
            self.add_overlay(const.GRID_TOPLEFT,
                             "Advance simulation by one step", "[right arrow]")
        self.add_overlay(const.GRID_TOPLEFT, "[H]ide Menu", "")
        if self._record_video:
            ndots = int(7 * (time.time() % 1))
            dots = ("." * ndots) + (" " * (6 - ndots))
            self.add_overlay(const.GRID_TOPLEFT,
                             "Record [V]ideo (On) " + dots, "")
        else:
            self.add_overlay(const.GRID_TOPLEFT, "Record [V]ideo (Off) ", "")
        if self._video_idx > 0:
            fname = self._video_path % (self._video_idx - 1)
            self.add_overlay(const.GRID_TOPLEFT, "   saved as %s" % fname, "")

        self.add_overlay(const.GRID_TOPLEFT, "Cap[t]ure frame", "")
        if self._image_idx > 0:
            fname = self._image_path % (self._image_idx - 1)
            self.add_overlay(const.GRID_TOPLEFT, "   saved as %s" % fname, "")
        self.add_overlay(const.GRID_TOPLEFT, "Start [i]pdb", "")
        if self._record_video:
            extra = " (while video is not recorded)"
        else:
            extra = ""
        self.add_overlay(const.GRID_BOTTOMLEFT, "FPS", "%d%s" %
                         (1 / self._time_per_render, extra))
        self.add_overlay(const.GRID_BOTTOMLEFT, "Solver iterations", str(
            self.sim.data.solver_iter + 1))
        step = round(self.sim.data.time / self.sim.model.opt.timestep)
        self.add_overlay(const.GRID_BOTTOMRIGHT, "Step", str(step))
        self.add_overlay(const.GRID_BOTTOMRIGHT, "timestep", "%.5f" % self.sim.model.opt.timestep)
        self.add_overlay(const.GRID_BOTTOMRIGHT, "n_substeps", str(self.sim.nsubsteps))
        self.add_overlay(const.GRID_TOPLEFT, "Toggle geomgroup visibility", "0-4")

    def key_callback(self, window, key, scancode, action, mods):
        if action != glfw.RELEASE:
            return
        elif key == glfw.KEY_TAB:  # Switches cameras.
            self.cam.fixedcamid += 1
            self.cam.type = const.CAMERA_FIXED
            if self.cam.fixedcamid >= self._ncam:
                self.cam.fixedcamid = -1
                self.cam.type = const.CAMERA_FREE
        elif key == glfw.KEY_H:  # hides all overlay.
            self._hide_overlay = not self._hide_overlay
        elif key == glfw.KEY_SPACE and self._paused is not None:  # stops simulation.
            self._paused = not self._paused
        # Advances simulation by one step.
        elif key == glfw.KEY_RIGHT and self._paused is not None:
            self._advance_by_one_step = True
            self._paused = True
        elif key == glfw.KEY_V or \
                (key == glfw.KEY_ESCAPE and self._record_video):  # Records video. Trigers with V or if in progress by ESC.
            self._record_video = not self._record_video
            if self._record_video:
                fps = (1 / self._time_per_render)
                self._video_process = Process(target=save_video,
                                  args=(self._video_queue, self._video_path % self._video_idx, fps))
                self._video_process.start()
            if not self._record_video:
                self._video_queue.put(None)
                self._video_process.join()
                self._video_idx += 1
        elif key == glfw.KEY_T:  # capture screenshot
            img = self._read_pixels_as_in_window()
            imageio.imwrite(self._image_path % self._image_idx, img)
            self._image_idx += 1
        elif key == glfw.KEY_I:  # drops in debugger.
            print('You can access the simulator by self.sim')
            import ipdb
            ipdb.set_trace()
        elif key == glfw.KEY_S:  # Slows down simulation.
            self._run_speed /= 2.0
        elif key == glfw.KEY_F:  # Speeds up simulation.
            self._run_speed *= 2.0
        elif key == glfw.KEY_C:  # Displays contact forces.
            vopt = self.vopt
            vopt.flags[10] = vopt.flags[11] = not vopt.flags[10]
        elif key == glfw.KEY_D:  # turn off / turn on rendering every frame.
            self._render_every_frame = not self._render_every_frame
        elif key == glfw.KEY_E:
            vopt = self.vopt
            vopt.frame = 1 - vopt.frame
        elif key == glfw.KEY_R:  # makes everything little bit transparent.
            self._transparent = not self._transparent
            if self._transparent:
                self.sim.model.geom_rgba[:, 3] /= 5.0
            else:
                self.sim.model.geom_rgba[:, 3] *= 5.0
        elif key == glfw.KEY_M:  # Shows / hides mocap bodies
            self._show_mocap = not self._show_mocap
            for body_idx1, val in enumerate(self.sim.model.body_mocapid):
                if val != -1:
                    for geom_idx, body_idx2 in enumerate(self.sim.model.geom_bodyid):
                        if body_idx1 == body_idx2:
                            if not self._show_mocap:
                                # Store transparency for later to show it.
                                self.sim.extras[
                                    geom_idx] = self.sim.model.geom_rgba[geom_idx, 3]
                                self.sim.model.geom_rgba[geom_idx, 3] = 0
                            else:
                                self.sim.model.geom_rgba[
                                    geom_idx, 3] = self.sim.extras[geom_idx]
        elif key in (glfw.KEY_0, glfw.KEY_1, glfw.KEY_2, glfw.KEY_3, glfw.KEY_4):
            self.vopt.geomgroup[key - glfw.KEY_0] ^= 1
        super().key_callback(window, key, scancode, action, mods)

# Separate Process to save video. This way visualization is
# less slowed down.


def save_video(queue, filename, fps):
    writer = imageio.get_writer(filename, fps=fps)
    while True:
        frame = queue.get()
        if frame is None:
            break
        writer.append_data(frame)
    writer.close()
