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


#include "egl.h"
#include <GL/glew.h>
#include <GL/gl.h>
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
void initOpenGL(void)
{
    // desired config
    const EGLint configAttribs[] ={
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

    // get default display
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if( eglDpy==EGL_NO_DISPLAY )
        mju_error_i("Could not get EGL display, error 0x%x\n", eglGetError());

    // initialize
    EGLint major, minor;
    if( eglInitialize(eglDpy, &major, &minor)!=EGL_TRUE )
        mju_error_i("Could not initialize EGL, error 0x%x\n", eglGetError());

    // choose config
    EGLint numConfigs;
    EGLConfig eglCfg;
    if( eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE )
        mju_error_i("Could not choose EGL config, error 0x%x\n", eglGetError());

    // bind OpenGL API
    if( eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE )
        mju_error_i("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());

    // create context
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
    if( eglCtx==EGL_NO_CONTEXT )
        mju_error_i("Could not create EGL context, error 0x%x\n", eglGetError());

    // make context current, no surface (let OpenGL handle FBO)
    if( eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE )
        mju_error_i("Could not make EGL context current, error 0x%x\n", eglGetError());
}


// close OpenGL context/window
void closeOpenGL(void)
{
    // get current display
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
    initOpenGL();
    initMuJoCo(argv[1]);

    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if( con.currentBuffer!=mjFB_OFFSCREEN )
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");

    // get size of active renderbuffer
    mjrRect viewport =  mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    // allocate rgb and depth buffers
//    unsigned char* rgb = (unsigned char*)malloc(3*W*H);
    unsigned char* rgb = 0;
    float* depth = (float*)malloc(sizeof(float)*W*H);
    if( !rgb || !depth )
        mju_error("Could not allocate buffers");

    // create output rgb file
    FILE* fp = fopen(argv[2], "wb");
    if( !fp )
        mju_error("Could not open rgbfile for writing");
//
//    // update abstract scene
//    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
//
//    // render scene in offscreen buffer
//    mjr_render(viewport, &scn, &con);

//    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, con.offFBO);

//    GLuint pixel_buffer = 0;
//    glGenBuffers(1, &pixel_buffer);
//    glBindBuffer(GL_PIXEL_PACK_BUFFER, pixel_buffer);
//    glBufferData(GL_PIXEL_PACK_BUFFER, 3*W*H, 0, GL_STREAM_READ);

//    glReadPixels(0, 0, W, H,
//                 GL_RGB, GL_UNSIGNED_BYTE, NULL);
//
//    rgb = (unsigned char *) glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
//
//    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
//    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
//    glDeleteBuffers(1, &pixel_buffer);

//    // read rgb and depth buffers
//    mjr_readPixels(rgb, depth, viewport, &con);
//
//    // insert subsampled depth image in lower-left corner of rgb image
//    const int NS = 3;           // depth image sub-sampling
//    for( int r=0; r<H; r+=NS )
//        for( int c=0; c<W; c+=NS )
//        {
//            int adr = (r/NS)*W + c/NS;
//            rgb[3*adr] = rgb[3*adr+1] = rgb[3*adr+2] =
//                (unsigned char)((1.0f-depth[r*W+c])*255.0f);
//        }
//
//    // write rgb image to file
//    fwrite(rgb, 3, W*H, fp);

    // close file, free buffers
    fclose(fp);
    free(rgb);
    free(depth);

    // close MuJoCo and OpenGL
    closeMuJoCo();
    closeOpenGL();

    return 0;
}