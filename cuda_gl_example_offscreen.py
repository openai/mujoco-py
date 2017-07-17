from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.pixel_buffer_object import *
from PIL import Image

from mujoco_py import OffscreenOpenGLContext


IMAGE_WIDTH, IMAGE_HEIGHT = 256, 256
framebuffer = None


def init_gl():
    glClearColor(0.1, 0.1, 0.5, 1.0)
    glDisable(GL_DEPTH_TEST)
    glViewport(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, IMAGE_WIDTH / IMAGE_HEIGHT, 0.1, 10.0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    glEnable(GL_LIGHT0)

    red = (1.0, 0.1, 0.1, 1.0)
    white = (1.0, 1.0, 1.0, 1.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red)
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white)
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 60.0)


def render_scene():
    # Ensure we're drawing to the FBO
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    glTranslatef(0.0, 0.0, -3.0)
    glRotatef(0.0, 1.0, 0.0, 0.0)
    glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(0.0, 0.0, 0.0, 1.0)
    glViewport(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)

    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)
    glutSolidTeapot(1.0)

    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer)

    buffer = glReadPixels(0, 0, 256, 256, GL_RGB, GL_UNSIGNED_BYTE)
    image = Image.frombuffer(
        mode="RGB", size=(256, 256), data=buffer, decoder_name="raw")
    # image = image.transpose(Image.FLIP_TOP_BOTTOM)
    image.save('framebuffer_read_pixels.png')


def create_framebuffer():
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glBlendEquation(GL_FUNC_SUBTRACT)

    render_buffer_color = glGenRenderbuffers(1)
    render_buffer_depth = glGenRenderbuffers(1)

    glBindRenderbuffer(GL_RENDERBUFFER, render_buffer_color)
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, IMAGE_WIDTH, IMAGE_HEIGHT)
    glBindRenderbuffer(GL_RENDERBUFFER, render_buffer_depth)
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                          IMAGE_WIDTH, IMAGE_HEIGHT)

    global framebuffer
    framebuffer = glGenFramebuffers(1)
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer)

    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, render_buffer_color)
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, render_buffer_depth)


def main():
    context = OffscreenOpenGLContext(device_id=0)

    init_gl()
    input("hellp")

    create_framebuffer()

    render_scene()

if __name__ == "__main__":
    main()
