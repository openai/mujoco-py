//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//

// to view:
// python -c "from PIL import Image; f=open('output.raw', 'rb'); d=f.read(); Image.frombytes(mode='RGB', size=(800, 800), data=d).save('output.png')"; open output.png

#include "mujoco.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


#define EGL_EGLEXT_PROTOTYPES
#include "egl.h"
#include "eglext.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

//-------------------------------- global data ------------------------------------------

// MuJoCo model and data
mjModel* m = 0;
mjData* d = 0;

// MuJoCo visualization
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;


//-------------------------------- utility functions ------------------------------------

// load model, init simulation and rendering
void initMuJoCo(const char* filename)
{
    // activate
    mj_activate("/root/.mujoco/mjkey.txt");

    // load and compile
    char error[1000] = "Could not load binary model";
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        m = mj_loadModel(filename, 0);
    else
        m = mj_loadXML(filename, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data, run one computation to initialize all fields
    d = mj_makeData(m);
    mj_forward(m, d);

    // initialize MuJoCo visualization
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, 200);

    // center and scale view
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
}


// deallocate everything and deactivate
void closeMuJoCo(void)
{
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deactivate();
}


// create OpenGL context/window
#define MAX_DEVICES 8
int is_device_initialized[MAX_DEVICES] = {0};
EGLDisplay eglDisplays[MAX_DEVICES];
EGLContext eglContexts[MAX_DEVICES];

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

    is_device_initialized[device_id] = 1;
    eglDisplays[device_id] = eglDpy;
    eglContexts[device_id] = eglCtx;
    return 1;
}


// close OpenGL context/window
void closeOpenGL()
{
    for (int device_id=0; device_id<MAX_DEVICES; device_id++) {
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


//-------------------------------- main function ----------------------------------------

int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=3 )
    {
        printf(" USAGE:  render_and_read modelfile rgbfile\n");
        return 0;
    }

    // initialize OpenGL and MuJoCo
    initOpenGL(1);
    initMuJoCo(argv[1]);

    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if( con.currentBuffer!=mjFB_OFFSCREEN )
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");

    // get size of active renderbuffer
    mjrRect viewport =  mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    // create output rgb file
    FILE* fp = fopen(argv[2], "wb");
    if( !fp )
        mju_error("Could not open rgbfile for writing");

    // update abstract scene
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // render scene in offscreen buffer
    mjr_render(viewport, &scn, &con);

    printf("Reading from framebuffer: %d\n", con.offFBO);
    printf("Number of samples: %d\n", con.offSamples);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        printf("ERROR: Framebuffer incomplete\n");

    GLubyte* src = (GLubyte*)malloc(3*W*H);
    if (con.offSamples == 0) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                     GL_RGB, GL_UNSIGNED_BYTE, src);
    } else {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
        glDrawBuffer(GL_COLOR_ATTACHMENT0);

        // resolve-blit
        glBlitFramebuffer(viewport.left, viewport.bottom,
                          viewport.left+viewport.width, viewport.bottom+viewport.height,
                          viewport.left, viewport.bottom,
                          viewport.left+viewport.width, viewport.bottom+viewport.height,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);

        // read from resolved
        glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO_r);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(viewport.left, viewport.bottom, viewport.width, viewport.height,
                     GL_RGB, GL_UNSIGNED_BYTE, src);
    }

    fwrite(src, 3, W*H, fp);
    int sum = 0;
    for (int i=0; i<(W*H*3); i++)
        sum += (int) src[i];
    printf("Sum of pixels: %d\n", sum);

    free(src);

//    GLuint pixel_buffer = 0;
//    glGenBuffers(1, &pixel_buffer);
//    glBindBuffer(GL_PIXEL_PACK_BUFFER_ARB, pixel_buffer);
//    glBufferData(GL_PIXEL_PACK_BUFFER_ARB, 3*W*H, 0, GL_STREAM_READ);
//    glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, 0);

//    GLubyte* rgb = (GLubyte*)glMapBufferARB(GL_PIXEL_PACK_BUFFER_ARB, GL_READ_ONLY_ARB);
//    unsigned char *rgb = (unsigned char *) glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
//
//    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
//    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
//    glDeleteBuffers(1, &pixel_buffer);

    // write rgb image to file
    printf("Done!\n");

    // close file, free buffers
    fclose(fp);

    // close MuJoCo and OpenGL
    closeMuJoCo();
    closeOpenGL();

    return 0;
}