'''
EXPERIMENTAL:  Displays MuJoCo model in VR.

Based on http://www.mujoco.org/book/source/mjvive.cpp
    No controller integration currently

This example is a demonstration of a near-direct translation of an existing
    MuJoCo example (mjvive.cpp), showing how to use lower-level functionality
    in conjunction with Python.

Known occasional issues with SteamVR (all-black textures seen),
    fixed by closing and re-opening.

Requires HTC Vive, Windows, and OpenVR.

Install openvr python module with:
    pip install openvr
'''


import math
import os
import glfw
import openvr
import numpy as np
import OpenGL.GL as gl
from mujoco_py import functions
from mujoco_py.builder import mujoco_path
from mujoco_py.cymj import (MjRenderContext, MjSim, load_model_from_xml,
                            PyMjrRect, PyMjvCamera)
from mujoco_py.generated.const import (CAT_ALL, FB_OFFSCREEN, FONT_BIG,
                                       GRID_BOTTOMLEFT, STEREO_SIDEBYSIDE)


# Normally global variables aren't used like this in python,
# but we want to be as close as possible to the original file.
class HMD():  # anonymous object we can set fields on
    pass


window = None
sim = None
ctx = None
hmd = HMD()


def initMuJoCo(filename, width2, height):
    ''' load model, init simulation and rendering '''
    global window, sim, ctx
    assert glfw.init(), 'Could not initialize GLFW'
    glfw.window_hint(glfw.SAMPLES, 0)
    glfw.window_hint(glfw.DOUBLEBUFFER, True)
    glfw.window_hint(glfw.RESIZABLE, 0)
    window = glfw.create_window(width2 // 4, height // 2, "mjvive.py", None, None)
    assert window, "Could not create GLFW window"
    glfw.make_context_current(window)
    glfw.swap_interval(0)
    # GLEW init required in C++, not in Python
    sim = MjSim(load_model_from_xml(open(filename).read()))
    sim.forward()
    sim.model.vis.global_.offwidth = width2
    sim.model.vis.global_.offheight = height
    sim.model.vis.quality.offsamples = 8
    ctx = MjRenderContext(sim)
    ctx.scn.enabletransform = 1
    ctx.scn.translate[1:3] = -0.5
    ctx.scn.rotate[0:2] = math.cos(-0.25 * math.pi), math.sin(-0.25 * math.pi)
    ctx.scn.scale = 1
    ctx.scn.stereo = STEREO_SIDEBYSIDE


def v_initPre():
    ''' init vr before MuJoCo init '''
    global hmd
    hmd.system = openvr.init(openvr.VRApplication_Scene)
    hmd.roompos = np.zeros(3)
    hmd.roommat = np.eye(3)
    hmd.eyeoffset = np.zeros((2, 3))
    openvr.VRCompositor().setTrackingSpace(openvr.TrackingUniverseStanding)
    hmd.width, hmd.height = hmd.system.getRecommendedRenderTargetSize()
    for n in range(2):
        hmd.eyeoffset[n] = np.array(hmd.system.getEyeToHeadTransform(n).m)[0:3, 3]


def v_initPost():
    ''' init vr after MuJoCo init '''
    global hmd
    for n in range(2):
        znear, zfar = 0.05, 50.0
        left, right, top, bottom = hmd.system.getProjectionRaw(n)
        ctx.scn.camera[n].frustum_bottom = -bottom * znear
        ctx.scn.camera[n].frustum_top = -top * znear
        ctx.scn.camera[n].frustum_center = 0.5 * (left + right) * znear
        ctx.scn.camera[n].frustum_near = znear
        ctx.scn.camera[n].frustum_far = zfar
    gl.glActiveTexture(gl.GL_TEXTURE2)
    hmd.idtex = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, hmd.idtex)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA8, 2 * hmd.width, hmd.height, 0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, None)
    hmd.poses = (openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount)()
    hmd.boundLeft = openvr.VRTextureBounds_t(0., 0., 0.5, 1.)
    hmd.boundRight = openvr.VRTextureBounds_t(0.5, 0., 1., 1.)
    hmd.vTex = openvr.Texture_t(hmd.idtex, openvr.TextureType_OpenGL, openvr.ColorSpace_Gamma)


def v_update():
    ''' update vr poses and controller states '''
    global ctx, hmd
    openvr.VRCompositor().waitGetPoses(hmd.poses, openvr.k_unMaxTrackedDeviceCount, None, 0)
    m = np.array(hmd.poses[openvr.k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking.m)
    hmd.roompos, hmd.roommat = m[0:3, 3], m[0:3, 0:3]
    for n in range(2):
        ctx.scn.camera[n].pos[:] = hmd.roompos + np.matmul(hmd.roommat, hmd.eyeoffset[n])
        ctx.scn.camera[n].forward[0:3] = -hmd.roommat[:, 2]
        ctx.scn.camera[n].up[0:3] = hmd.roommat[:, 1]


def v_render():
    ''' render to vr and window '''
    global hmd, ctx, window
    # resolve multi-sample offscreen buffer
    gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, ctx.con.offFBO)
    gl.glReadBuffer(gl.GL_COLOR_ATTACHMENT0)
    gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, ctx.con.offFBO_r)
    gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
    gl.glBlitFramebuffer(0, 0, 2 * hmd.width, hmd.height,
                         0, 0, 2 * hmd.width, hmd.height,
                         gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
    # blit to window, left only, window is half-size
    gl.glBindFramebuffer(gl.GL_READ_FRAMEBUFFER, ctx.con.offFBO_r)
    gl.glReadBuffer(gl.GL_COLOR_ATTACHMENT0)
    gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, 0)
    gl.glDrawBuffer(gl.GL_BACK if ctx.con.windowDoublebuffer else gl.GL_FRONT)
    gl.glBlitFramebuffer(0, 0, hmd.width, hmd.height,
                         0, 0, hmd.width // 2, hmd.height // 2,
                         gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
    # blit to vr texture
    gl.glActiveTexture(gl.GL_TEXTURE2)
    gl.glBindFramebuffer(gl.GL_DRAW_FRAMEBUFFER, ctx.con.offFBO_r)
    gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT1, gl.GL_TEXTURE_2D, hmd.idtex, 0)
    gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT1)
    gl.glBlitFramebuffer(0, 0, 2 * hmd.width, hmd.height,
                         0, 0, 2 * hmd.width, hmd.height,
                         gl.GL_COLOR_BUFFER_BIT, gl.GL_NEAREST)
    gl.glFramebufferTexture2D(gl.GL_FRAMEBUFFER, gl.GL_COLOR_ATTACHMENT1, gl.GL_TEXTURE_2D, 0, 0)
    gl.glDrawBuffer(gl.GL_COLOR_ATTACHMENT0)
    openvr.VRCompositor().submit(openvr.Eye_Left, hmd.vTex, hmd.boundLeft)
    openvr.VRCompositor().submit(openvr.Eye_Right, hmd.vTex, hmd.boundRight)
    # swap if window is double-buffered, flush just in case
    if ctx.con.windowDoublebuffer:
        glfw.swap_buffers(window)
    gl.glFlush()


if __name__ == '__main__':
    filename = os.path.join(mujoco_path, 'model', 'humanoid.xml')
    v_initPre()
    initMuJoCo(filename, hmd.width * 2, hmd.height)
    v_initPost()

    FPS = 90.0
    lasttm = glfw.get_time()
    frametime = sim.data.time
    viewFull = PyMjrRect()
    viewFull.width, viewFull.height = 2 * hmd.width, hmd.height
    nullCam = PyMjvCamera()

    while not glfw.window_should_close(window):
        if sim.data.time - frametime > 1 / FPS or sim.data.time < frametime:
            functions.mjv_updateScene(sim.model, sim.data, ctx.vopt, ctx.pert, nullCam, CAT_ALL, ctx.scn)
            v_update()
            functions.mjr_setBuffer(FB_OFFSCREEN, ctx.con)
            functions.mjr_render(viewFull, ctx.scn, ctx.con)
            FPS = .9 * FPS + .1 / (glfw.get_time() - lasttm)
            lasttm = glfw.get_time()
            functions.mjr_overlay(FONT_BIG, GRID_BOTTOMLEFT, viewFull, 'FPS %.1f' % FPS, '', ctx.con)
            v_render()
            frametime = sim.data.time
        sim.step()
        glfw.poll_events()
    # close
    openvr.shutdown()
    gl.glDeleteTextures(1, hmd.idtex)
    del sim
    del ctx
    glfw.terminate()
