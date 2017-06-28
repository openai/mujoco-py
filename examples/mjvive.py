# Displays MuJoCo model in VR.
# This code is a Python port of http://www.mujoco.org/book/source/mjvive.cpp
# Dependencies: Windows 7 with SteamVR, HTC Vive.
# Install py-openvr with `pip install openvr`.
# NOTE: THIS CODE IS EXPERIMENTAL.

import math
import sys
import glfw
import OpenGL.GL as gl
import openvr
import numpy as np
import OpenGL.GL as gl
import os
from mujoco_py import MjSim, load_model_from_xml, const, functions
from mujoco_py.builder import cymj


class MjViewerVR(cymj.MjRenderContext):
    def __init__(self, sim):
        self.FPS = 90
        print("bbb")
        self.v_initPre()
        print("aaa")
        super().__init__(sim)
        print("ccc")
        self.initMuJoCo()
        self.v_initPost()
        self.lasttm = glfw.get_time()
        self.frametime = self.sim.data.time

    # def _setup_opengl_context(self, *args):
    #     print("_setup_opengl_context")

    # def _set_mujoco_buffers(self):
        # print("_set_mujoco_buffers")

    def _init_camera(self, *args):
        print("_init_camera")


    def v_initPre(self):
        self.system = openvr.init(openvr.VRApplication_Scene)
        self.roompos = np.zeros(3)
        self.roommat = np.eye(3)
        self.eyeoffset = np.zeros((2, 3))
        openvr.VRCompositor().setTrackingSpace(openvr.TrackingUniverseStanding)
        self.width, self.height = self.system.getRecommendedRenderTargetSize()
        for n in range(2):
            self.eyeoffset[n] = np.array(self.system.getEyeToHeadTransform(n).m)[0:3, 3]
        self.rect = cymj.mujoco_pyrRect()
        self.rect.left, self.rect.bottom = 0, 0
        self.rect.width, self.rect.height = 2 * self.width, self.height

    def initMuJoCo(self):
        assert glfw.init(), 'Could not initialize GLFW'
        glfw.window_hint(glfw.SAMPLES, 0)
        glfw.window_hint(glfw.DOUBLEBUFFER, 1)
        glfw.window_hint(glfw.RESIZABLE, 0)
        self._window = glfw.create_window(self.width // 2, self.height // 2, "MuJoCo VR", None, None)
        assert self._window, "Could not create GLFW window"
        glfw.make_context_current(self._window)
        glfw.swap_interval(0)
        # GLEW init required on windows, not in python
        self.sim.forward()
        self.sim.model.vis.global_.offwidth = self.width * 2
        self.sim.model.vis.global_.offheight = self.height
        self.sim.model.vis.quality.offsamples = 8
        self.scn.enabletransform = 1
        self.scn.translate[1:3] = -0.5
        self.scn.rotate[0:2] = math.cos(-0.25 * math.pi), math.sin(-0.25 * math.pi)
        self.scn.scale = 1
        self.scn.stereo = const.STEREO_SIDEBYSIDE

    def v_initPost(self):
        for n in range(2):
            znear, zfar = 0.05, 50.0
            left, right, top, bottom = self.system.getProjectionRaw(n)
            self.scn.camera[n].frustum_bottom = -bottom * znear
            self.scn.camera[n].frustum_top = -top * znear
            self.scn.camera[n].frustum_center = 0.5 * (left + right) * znear
            self.scn.camera[n].frustum_near = znear
            self.scn.camera[n].frustum_far = zfar
        gl.glActiveTexture(gl.GL_TEXTURE2)
        self.idtex = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.idtex)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA8, 2 * self.width, self.height, 0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, None)

    def v_update(self):
        poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
        openvr.VRCompositor().waitGetPoses(poses, openvr.k_unMaxTrackedDeviceCount, None, 0)
        m = np.array(poses[openvr.k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking.m)
        self.roompos, self.roommat = m[0:3, 3], m[0:3, 0:3]
        for n in range(2):
            self.scn.camera[n].pos[:] = self.roompos + np.matmul(self.roommat, self.eyeoffset[n])
            self.scn.camera[n].forward[0:3] = -self.roommat[:, 2]
            self.scn.camera[n].up[0:3] = self.roommat[:, 1]

    def v_render(self):
        # resolve multi-sample offscreen buffer
        gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, self.con.offFBO)
        gl.glReadBuffer(gl.GL_COLOR_ATTACHMENT0)
        gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, self.con.offFBO_r)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        gl.glBlitFramebuffer(0, 0, 2 * self.width, self.height,
                             0, 0, 2 * self.width, self.height,
                             gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
        # blit to window, left only, window is half-size
        gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, self.con.offFBO_r)
        gl.glReadBuffer(gl.GL_COLOR_ATTACHMENT0)
        gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, 0)
        gl.glDrawBuffer(gl.GL_BACK if self.con.windowDoublebuffer else gl.glDrawBuffer(gl.GL_FRONT))
        gl.glBlitFramebuffer(0, 0, self.width, self.height,
                             0, 0, self.width // 2, self.height // 2,
                             gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
        # blit to vr texture
        gl.glActiveTexture(gl.GL_TEXTURE2)
        gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, self.con.offFBO_r)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT1, gl.GL_TEXTURE_2D, self.idtex, 0)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT1)
        gl.glBlitFramebuffer(0, 0, 2 * self.width, self.height,
                             0, 0, 2 * self.width, self.height,
                             gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
        gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT1, gl.GL_TEXTURE_2D, 0, 0)
        gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
        # submit to vr
        boundLeft = openvr.VRTextureBounds_t(0., 0., 0.5, 1.)
        boundRight = openvr.VRTextureBounds_t(0.5, 0., 1., 1.)
        vTex = openvr.Texture_t(self.idtex, openvr.TextureType_OpenGL, openvr.ColorSpace_Gamma)
        openvr.VRCompositor().submit(openvr.Eye_Left, vTex, boundLeft)
        openvr.VRCompositor().submit(openvr.Eye_Right, vTex, boundRight)
        # swap if window is double-buffered, flush just in case
        if self.con.windowDoublebuffer:
            glfw.swap_buffers(self._window)
        gl.glFlush()

    def render(self):
        while not glfw.window_should_close(self._window):
            if self.sim.data.time - self.frametime > 1 / self.FPS or self.sim.data.time < self.frametime:
                print("aaa")
                functions.mjv_updateScene(self.sim.model, self.sim.data, self.vopt, self.pert, self.cam, const.CAT_ALL, self.scn)
                print("qqqwe")
                self.v_update()
                print("dasdasdsaasd")
                functions.mjr_setBuffer(const.FB_OFFSCREEN, self.con)
                self._opengl_context.set_buffer_size(self.width, self.height)
                print("111")
                functions.mjr_render(self.rect, self.scn, self.con)
                print("222")
                self.FPS = .9 * self.FPS + .1 / (glfw.get_time() - self.lasttm)
                self.lasttm = glfw.get_time()
                self.v_render()
                self.frametime = self.sim.data.time
            self.sim.step()
            glfw.poll_events()


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        fname = sys.argv[1]
    else:
        fname = os.path.join(os.path.expanduser("~"), ".mujoco", "mjpro150", "model", "humanoid100.xml")

    sim = MjSim(load_model_from_xml(open(fname).read()))
    viewer = MjViewerVR(sim)

    # XXX: here should be while loop.
    viewer.render()
