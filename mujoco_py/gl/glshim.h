#ifndef __GLSHIM_H__
#define __GLSHIM_H__

#include "mujoco.h"
#include "mjrender.h"

int initOpenGL(int device_id);
void closeOpenGL();
int makeOpenGLContextCurrent(int device_id);
int setOpenGLBufferSize(int device_id, int width, int height);

unsigned int createPBO(int width, int height, int batchSize);
void freePBO(unsigned int pixelBuffer);
void copyFBOToPBO(mjrContext* con, unsigned int pbo,
                  mjrRect viewport, int bufferOffset);
void readPBO(unsigned char *buffer, unsigned int pbo,
             int width, int height, int batchSize);

long int getCurrentOpenGLContext(int device_id);
long int getCurrentOpenGLDisplay(int device_id);

#endif
