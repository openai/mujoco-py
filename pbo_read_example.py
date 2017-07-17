from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL.ARB.vertex_buffer_object import *
from OpenGL.GL.ARB.pixel_buffer_object import *
from ctypes import sizeof, c_float, c_void_p, c_uint, string_at
import time
import numpy

initial_size = 512,512
current_size = initial_size
animate = True
enable_cuda = True
window = None     # Number of the glut window.
time_of_last_draw = 0.0
time_of_last_titleupdate = 0.0
frames_per_second = 0.0
frame_counter = 0
output_texture = None # pointer to offscreen render target
(source_pbo, dest_pbo, cuda_module, invert,
 pycuda_source_pbo, pycuda_dest_pbo) = [None]*6
heading,pitch,bank = [0.0]*3

def create_PBOs(w,h):
    global source_pbo, dest_pbo
    num_texels = w*h
    data = numpy.zeros((num_texels,4),numpy.uint8)
    source_pbo = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, source_pbo)
    glBufferData(GL_ARRAY_BUFFER, data, GL_DYNAMIC_DRAW)
    glBindBuffer(GL_ARRAY_BUFFER, 0)

    dest_pbo = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, dest_pbo)
    glBufferData(GL_ARRAY_BUFFER, data, GL_DYNAMIC_DRAW)
    glBindBuffer(GL_ARRAY_BUFFER, 0)

def destroy_PBOs():
    global source_pbo, dest_pbo
    for pbo in [source_pbo, dest_pbo]:
        glBindBuffer(GL_ARRAY_BUFFER, long(pbo))
        glDeleteBuffers(1, long(pbo));
        glBindBuffer(GL_ARRAY_BUFFER, 0)

def create_texture(w,h):
    global output_texture
    output_texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, output_texture)
    # set basic parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    # buffer data
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)

def destroy_texture():
    global output_texture
    glDeleteTextures(output_texture);
    output_texture = None

def init_gl():
    Width, Height = current_size
    glClearColor(0.1, 0.1, 0.5, 1.0)
    glDisable(GL_DEPTH_TEST)
    glViewport(0, 0, Width, Height)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, Width/float(Height), 0.1, 10.0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    glEnable(GL_LIGHT0)
    red   = ( 1.0, 0.1, 0.1, 1.0 )
    white = ( 1.0, 1.0, 1.0, 1.0 )
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,  red  )
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white)
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 60.0)

def resize(Width, Height):
    global current_size
    current_size = Width, Height
    glViewport(0, 0, Width, Height)        # Reset The Current Viewport And Perspective Transformation
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60.0, Width/float(Height), 0.1, 10.0)

def do_tick():
    global time_of_last_titleupdate, frame_counter, frames_per_second
    if ((time.clock () * 1000.0) - time_of_last_titleupdate >= 1000.):
        frames_per_second = frame_counter                   # Save The FPS
        frame_counter = 0  # Reset The FPS Counter
        szTitle = "%d FPS" % (frames_per_second )
        glutSetWindowTitle ( szTitle )
        time_of_last_titleupdate = time.clock () * 1000.0
    frame_counter += 1

# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)
def keyPressed(*args):
    global animate, enable_cuda
    # If escape is pressed, kill everything.
    if args[0] == '\033':
        print('Closing..')
        destroy_PBOs()
        destroy_texture()
        exit()
    elif args[0] == 'a':
        print('toggling animation')
        animate = not animate

def idle():
    global heading, pitch, bank
    if animate:
        heading += 0.2
        pitch   += 0.6
        bank    += 1.0

    glutPostRedisplay()

def display():
    try:
        render_scene()
        # process_image()
        screenshot()
        glutSwapBuffers()
    except:
        from traceback import print_exc
        print_exc()
        from os import _exit
        _exit(0)

screenshot_done = False
def screenshot():
    global screenshot_done
    if screenshot_done:
        return
    width, height = current_size
    size = width * height * 3
    pixel_buffer = glGenBuffers(1)
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffer)
    glBufferData(GL_PIXEL_PACK_BUFFER, size, None, GL_STREAM_READ)

    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, c_void_p(0))
    buffer = string_at(glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY), size)

    from PIL import Image
    image = Image.frombuffer(
        mode="RGB", size=current_size, data=buffer, decoder_name="raw")
    image.save('pbo_map_example.png')

    glUnmapBuffer(GL_PIXEL_PACK_BUFFER)
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0)
    glDeleteBuffers(1, [pixel_buffer])
    screenshot_done = True

def process_image():
    """ copy image and process using CUDA """
    global source_pbo,current_size, dest_pbo
    image_width, image_height = current_size
    assert source_pbo is not None

    # activate destination buffer
    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, long(source_pbo))

    # read data into pbo. note: use BGRA format for optimal performance
    glReadPixels(
             0,                  #start x
             0,                  #start y
             image_width,        #end   x
             image_height,       #end   y
             GL_BGRA,            #format
             GL_UNSIGNED_BYTE,   #output type
             ctypes.c_void_p(0))

    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0)

    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, long(source_pbo))
    glMapBuffer(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY)

    glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0)


def display_image():
    """ render a screen sized quad """
    glDisable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_TEXTURE_2D)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE)
    glMatrixMode(GL_PROJECTION)
    glPushMatrix()
    glLoadIdentity()
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
    glMatrixMode( GL_MODELVIEW)
    glLoadIdentity()
    glViewport(0, 0, current_size[0], current_size[1])
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 0.0)
    glVertex3f(-1.0, -1.0, 0.5)
    glTexCoord2f(1.0, 0.0)
    glVertex3f(1.0, -1.0, 0.5)
    glTexCoord2f(1.0, 1.0)
    glVertex3f(1.0, 1.0, 0.5)
    glTexCoord2f(0.0, 1.0)
    glVertex3f(-1.0, 1.0, 0.5)
    glEnd()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glDisable(GL_TEXTURE_2D)
    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0)
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER_ARB, 0)


def render_scene():
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)# Clear Screen And Depth Buffer
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity ()      # Reset The Modelview Matrix
    glTranslatef(0.0, 0.0, -3.0);
    glRotatef(heading, 1.0, 0.0, 0.0)
    glRotatef(pitch  , 0.0, 1.0, 0.0)
    glRotatef(bank   , 0.0, 0.0, 1.0)
    glViewport(0, 0, current_size[0],current_size[1])
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)
    glutSolidTeapot(1.0)
    do_tick()#just for fps display..
    return True

def main():
    global window, cuda_module, cuda_gl, cuda_driver, invert
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(*initial_size)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("PyCuda GL Interop Example")
    glutDisplayFunc(display)
    glutIdleFunc(idle)
    glutReshapeFunc(resize)
    glutKeyboardFunc(keyPressed)
    glutSpecialFunc(keyPressed)
    init_gl()

    # create texture for blitting to screen
    create_texture(*initial_size)

    # create source and destination pixel buffer objects for processing
    create_PBOs(*initial_size)

    glutMainLoop()

# Print message to console, and kick off the main to get it rolling.
if __name__ == "__main__":
    print("Hit ESC key to quit, 'a' to toggle animation, and 'e' to toggle cuda")
    main()