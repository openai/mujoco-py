#ifndef __GLSHIM_H__
#define __GLSHIM_H__

#include "mujoco.h"
#include "mjrender.h"

#ifdef __cplusplus
extern "C" {
#endif

int usingEGL();
int initOpenGL(int device_id);
void closeOpenGL();
int makeOpenGLContextCurrent(int device_id);
int setOpenGLBufferSize(int device_id, int width, int height);

unsigned int createPBO(int width, int height, int batchSize, int use_short);
void freePBO(unsigned int pixelBuffer);
void copyFBOToPBO(mjrContext* con,
                  unsigned int pbo_rgb, unsigned int pbo_depth,
                  mjrRect viewport, int bufferOffset);
void readPBO(unsigned char *buffer_rgb, unsigned short *buffer_depth,
             unsigned int pbo_rgb, unsigned int pbo_depth,
             int width, int height, int batchSize);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
