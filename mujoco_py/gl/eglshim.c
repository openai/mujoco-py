#define EGL_EGLEXT_PROTOTYPES
#include "egl.h"
#include "eglext.h"
#include <GL/glew.h>

#include "mujoco.h"
#include "mjrender.h"

#include "glshim.h"

#define MAX_DEVICES 8

int is_device_initialized[MAX_DEVICES] = {0};
EGLDisplay eglDisplays[MAX_DEVICES];
EGLContext eglContexts[MAX_DEVICES];

int usingEGL() {
    return 1;
}

int initOpenGL(int device_id)
{
    if (device_id < 0 || device_id > MAX_DEVICES) {
        printf("Device id outside of range.\n");
        return -1;
    }
    int is_initialized = is_device_initialized[device_id];

    if (is_initialized)
        return 1;

    // desired config
    const EGLint configAttribs[] = {
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         16,
        EGL_STENCIL_SIZE,       8,
        EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_NONE
    };

    EGLDeviceEXT eglDevs[MAX_DEVICES];
    EGLint numDevices;

    PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
        (PFNEGLQUERYDEVICESEXTPROC)
        eglGetProcAddress("eglQueryDevicesEXT");

    eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);
    printf("Found %d GPUs for rendering. Using device %d.\n", numDevices, device_id);
    if (device_id >= numDevices) {
        printf("Device id outside of range of available devices.\n");
        return -1;
    }

    PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
    (PFNEGLGETPLATFORMDISPLAYEXTPROC)
    eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (eglGetPlatformDisplayEXT == NULL) {
        printf("Failed to get eglGetPlatformDisplayEXT\n");
        return -2;
    }

    EGLDisplay eglDpy = eglGetPlatformDisplayEXT(
        EGL_PLATFORM_DEVICE_EXT, eglDevs[device_id], 0);

    // get default display
    // EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (eglDpy == EGL_NO_DISPLAY) {
        printf("Could not get EGL display\n");
        return -3;
    }

    // initialize
    EGLint major, minor;
    if (eglInitialize(eglDpy, &major, &minor) != EGL_TRUE) {
        printf("Could not initialize EGL\n");
        return -4;
    }

    // choose config
    EGLint numConfigs;
    EGLConfig eglCfg;
    if (eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE ) {
        printf("Could not choose EGL config\n");
        return -5;
    }

    // bind OpenGL API
    if( eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE ) {
        printf("Could not bind EGL OpenGL API\n");
        return -6;
    }

    // create context
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
    if( eglCtx==EGL_NO_CONTEXT ) {
        printf("Could not create EGL context\n");
        return -7;
    }

    // make context current, no surface (let OpenGL handle FBO)
    if( eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE ) {
        eglDestroyContext(eglDpy, eglCtx);
        printf("Could not make EGL context current\n");
        return -8;
    }

    GLenum err = glewInit();
    if( GLEW_OK != err ) {
        // MuJoCo does this automatically, but we need it if we want
        // to create e.g. PBOs before calling MuJoCo rendering functions.
        fprintf("glewInit error: %s\n", glewGetErrorString(err));
        return -9;
    }

    is_device_initialized[device_id] = 1;
    eglDisplays[device_id] = eglDpy;
    eglContexts[device_id] = eglCtx;

    return 1;
}

int makeOpenGLContextCurrent(int device_id) {
    if (device_id < 0 || device_id > MAX_DEVICES) {
        printf("Device id outside of range.\n");
        return -1;
    }
    if (!is_device_initialized[device_id])
        return -2;

    if( eglMakeCurrent(eglDisplays[device_id],
                       EGL_NO_SURFACE,
                       EGL_NO_SURFACE,
                       eglContexts[device_id]) != EGL_TRUE ) {
        eglDestroyContext(eglDisplays[device_id],
                          eglContexts[device_id]);
        printf("Could not make EGL context current\n");
        return -3;
    } else {
        return 1;
    }
}

int setOpenGLBufferSize(int device_id, int width, int height) {
    // Noop since we don't need to change buffer here.
    return 1;
}

void closeOpenGL()
{
    int device_id;
    for (device_id=0; device_id<MAX_DEVICES; device_id++) {
        if (!is_device_initialized[device_id])
            continue;

        EGLDisplay eglDpy = eglDisplays[device_id];
        if( eglDpy==EGL_NO_DISPLAY )
            continue;

        // get current context
        EGLContext eglCtx = eglContexts[device_id];

        // release context
        eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

        // destroy context if valid
        if( eglCtx!=EGL_NO_CONTEXT )
            eglDestroyContext(eglDpy, eglCtx);

        // terminate display
        eglTerminate(eglDpy);
    }
}

// Create Pixel Buffer Objects (PBO) for rgb and depth image, which can be
// mapped to the CPU directly as a big batch.
unsigned int createPBO(int width, int height, int batchSize, int use_short)
{
    GLuint pixelBuffer = 0;
    glGenBuffers(1, &pixelBuffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pixelBuffer);

    GLsizeiptr buffer_size;
    if (use_short) {
        buffer_size = batchSize * width * height * sizeof(short);
    } else {
        buffer_size = batchSize * width * height * 3;
    }
    glBufferData(GL_PIXEL_PACK_BUFFER_ARB, buffer_size, 0, GL_DYNAMIC_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);

    return (unsigned int) pixelBuffer;
}

void freePBO(unsigned int pixelBuffer)
{
    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
    glDeleteBuffers(1, &pixelBuffer);
}

void readPBO(unsigned char *buffer_rgb, unsigned short *buffer_depth,
             unsigned int pbo_rgb, unsigned int pbo_depth,
             int width, int height, int batchSize)
{
    if (pbo_rgb > 0) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo_rgb);

        GLubyte* src_rgb = (GLubyte*) glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
        memcpy(buffer_rgb, src_rgb, batchSize * width * height * 3);

        glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }
    if (pbo_depth > 0) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo_depth);

        GLushort* src_depth = (GLushort*) glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
        memcpy(buffer_depth, src_depth, batchSize * width * height * sizeof(GLushort));

        glUnmapBuffer(GL_PIXEL_PACK_BUFFER_ARB);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
}

// Copy the render data from the Framebuffer Object (FBO) that Mujoco
// uses for offscreen rendering into the PBOs.
void copyFBOToPBO(mjrContext* con,
                  unsigned int pbo_rgb, unsigned int pbo_depth,
                  mjrRect viewport, int bufferOffset)
{
    GLbitfield mask = (pbo_rgb ? GL_COLOR_BUFFER_BIT : 0) |
                      (pbo_depth ? GL_DEPTH_BUFFER_BIT : 0);

    if (!mask)
        return;

    // multisample: blit to resolve buffer and read from there
    if (con->offSamples)
    {
        // make sure blit is supported
        if( !glBlitFramebuffer )
            return;

        // prepare for resolve-blit
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con->offFBO_r);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);

        // resolve-blit
        glBlitFramebuffer(viewport.left, viewport.bottom,
                          viewport.left + viewport.width, viewport.bottom + viewport.height,
                          viewport.left, viewport.bottom,
                          viewport.left + viewport.width, viewport.bottom + viewport.height,
                          mask, GL_NEAREST);

        // read from resolved
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO_r);
    }
    // no multisample: read from offscreen
    else
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);

    glReadBuffer(GL_COLOR_ATTACHMENT0);
    if (pbo_rgb) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo_rgb);
        glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                     GL_RGB, GL_UNSIGNED_BYTE,
                     bufferOffset * viewport.width * viewport.height * 3);
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
    }
    if (pbo_depth) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pbo_depth);
        glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                     GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT,
                     bufferOffset * viewport.width * viewport.height * sizeof(short));
        glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, 0);
    }

    // restore currentBuffer
    glBindFramebuffer(GL_FRAMEBUFFER, con->offFBO);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
}
