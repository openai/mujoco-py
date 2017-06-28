#define EGL_EGLEXT_PROTOTYPES
#include "egl.h"
#include "eglext.h"
#include "mujoco.h"
#include "glshim.h"

#define MAX_DEVICES 8

int is_initialized = 0;

int initOpenGL(int device_id)
{
    if (is_initialized)
        return 1;

    // desired config
    const EGLint configAttribs[] = {
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         24,
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

    is_initialized = 1;
    return 1;
}

int setOpenGLBufferSize(int width, int height) {
    // Noop since we don't need to change buffer here.
    return 1;
}

void closeOpenGL()
{
    if (!is_initialized)
        return;

    EGLDisplay eglDpy = eglGetCurrentDisplay();
    if( eglDpy==EGL_NO_DISPLAY )
        return;

    // get current context
    EGLContext eglCtx = eglGetCurrentContext();

    // release context
    eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    // destroy context if valid
    if( eglCtx!=EGL_NO_CONTEXT )
        eglDestroyContext(eglDpy, eglCtx);

    // terminate display
    eglTerminate(eglDpy);
}
