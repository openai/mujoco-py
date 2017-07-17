#!/usr/bin/env python3
from ctypes import c_void_p, string_at
import OpenGL.GL as gl
import OpenGL.GL.framebufferobjects as gl_fbo
from PIL import Image
from mujoco_py import load_model_from_path, MjSim


def save_image_from_pbo(width, height, fbo_id):
    # gl_fbo.glBindFramebufferEXT(gl_fbo.GL_FRAMEBUFFER, fbo_id)

    buffer_size = width * height * 3
    pixel_buffer = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_PIXEL_PACK_BUFFER, pixel_buffer)
    gl.glBufferData(gl.GL_PIXEL_PACK_BUFFER, buffer_size, None, gl.GL_STREAM_READ)

    gl.glReadPixels(0, 0, width, height,
                    gl.GL_RGB, gl.GL_UNSIGNED_BYTE, c_void_p(0))
    buffer = string_at(
        gl.glMapBuffer(gl.GL_PIXEL_PACK_BUFFER, gl.GL_READ_ONLY), buffer_size)

    image = Image.frombuffer(
        mode="RGB", size=(width, height), data=buffer, decoder_name="raw")
    image.save('fbo_to_pbo_example.png')

    gl.glUnmapBuffer(gl.GL_PIXEL_PACK_BUFFER)
    gl.glBindBuffer(gl.GL_PIXEL_PACK_BUFFER, 0)
    gl.glDeleteBuffers(1, [pixel_buffer])


def main():
    model = load_model_from_path("xmls/fetch/main.xml")
    sim = MjSim(model)

    im_w, im_h = 256, 256
    sim.render(im_w, im_h)
    rc = sim.render_contexts[0]

    save_image_from_pbo(im_w, im_h, rc.con.offFBO)


if __name__ == '__main__':
    main()
